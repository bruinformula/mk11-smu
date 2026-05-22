#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import Counter
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

NMEA_SENTENCE_RE = re.compile(r"\$[A-Z0-9]{5},[^\r\n$]{0,160}?\*[0-9A-Fa-f]{2}")
GGA_FRAGMENT_RE = re.compile(
    r"(?P<time>\d{3,6}\.\d+),(?P<lat>\d{4,5}\.\d+),(?P<lat_hemi>[NS]),"
    r"(?P<lon>\d{5,6}\.\d+),(?P<lon_hemi>[EW]),(?P<quality>\d),"
    r"(?P<satellites>\d{1,2}),(?P<hdop>-?\d+(?:\.\d+)?),(?P<altitude>-?\d+(?:\.\d+)?),M,"
    r"(?P<geoid>-?\d+(?:\.\d+)?),M"
)
RMC_TYPES = {"GNRMC", "GPRMC", "GARMC", "GLRMC", "GBRMC", "GQRMC"}
GSV_TYPES = {"GPGSV", "GLGSV", "GAGSV", "GBGSV", "GQGSV", "GNGSV"}
GSA_TYPES = {"GNGSA", "GPGSA", "GLGSA", "GAGSA", "GBGSA", "GQGSA"}
KNOTS_TO_MPS = 0.514444
LOCK_QUALITY_LABELS = {
    0: "Invalid",
    1: "GPS fix",
    2: "DGPS fix",
    3: "PPS fix",
    4: "RTK fixed",
    5: "RTK float",
    6: "Estimated",
    7: "Manual",
    8: "Simulation",
}


@dataclass
class TrackPoint:
    latitude: float
    longitude: float
    timestamp: Optional[str]
    speed_knots: Optional[float]
    speed_mps: Optional[float]
    course_deg: Optional[float]
    altitude_m: Optional[float]
    gps_lock_quality: Optional[int]
    gps_lock_quality_label: Optional[str]
    satellite_count_used: Optional[int]
    satellite_count_in_view: Optional[int]
    average_snr_dbhz: Optional[float]
    max_snr_dbhz: Optional[float]
    hdop: Optional[float]
    longitudinal_accel_mps2: Optional[float]
    latitudinal_accel_mps2: Optional[float]
    accel_magnitude_mps2: Optional[float]
    sentence_type: str
    raw_sentence: str


class ParserError(Exception):
    pass


@dataclass
class QualityContext:
    lock_quality: Optional[int] = None
    lock_quality_label: Optional[str] = None
    satellite_count_used: Optional[int] = None
    satellite_count_in_view: Optional[int] = None
    average_snr_dbhz: Optional[float] = None
    max_snr_dbhz: Optional[float] = None
    hdop: Optional[float] = None
    altitude_m: Optional[float] = None


@dataclass
class GsvTalkerState:
    total_messages: int
    total_satellites: int
    parts: dict[int, list[float]]


def validate_checksum(sentence: str) -> bool:
    if "*" not in sentence or not sentence.startswith("$"):
        return False

    payload, checksum_text = sentence[1:].split("*", 1)
    checksum = 0
    for character in payload:
        checksum ^= ord(character)

    try:
        expected_checksum = int(checksum_text[:2], 16)
    except ValueError:
        return False

    return checksum == expected_checksum


def extract_sentences(raw_bytes: bytes) -> tuple[list[str], int]:
    decoded = raw_bytes.decode("latin-1")
    sentences: list[str] = []
    invalid_checksum_count = 0

    for match in NMEA_SENTENCE_RE.finditer(decoded):
        sentence = match.group(0)
        if validate_checksum(sentence):
            sentences.append(sentence)
        else:
            invalid_checksum_count += 1

    return sentences, invalid_checksum_count


def iter_valid_sentences(line: str) -> tuple[list[str], int]:
    sentences: list[str] = []
    invalid_checksum_count = 0

    for match in NMEA_SENTENCE_RE.finditer(line):
        sentence = match.group(0)
        if validate_checksum(sentence):
            sentences.append(sentence)
        else:
            invalid_checksum_count += 1

    return sentences, invalid_checksum_count


def parse_coordinate(value: str, hemisphere: str) -> Optional[float]:
    if not value or not hemisphere:
        return None

    if hemisphere not in {"N", "S", "E", "W"}:
        return None

    degree_digits = 2 if hemisphere in {"N", "S"} else 3
    if len(value) <= degree_digits:
        return None

    try:
        degrees = float(value[:degree_digits])
        minutes = float(value[degree_digits:])
    except ValueError:
        return None

    decimal = degrees + minutes / 60.0
    if hemisphere in {"S", "W"}:
        decimal *= -1
    return decimal


def parse_float(value: str) -> Optional[float]:
    if not value:
        return None

    try:
        return float(value)
    except ValueError:
        return None


def parse_int(value: str) -> Optional[int]:
    if not value:
        return None

    try:
        return int(value)
    except ValueError:
        return None


def parse_timestamp(date_value: str, time_value: str) -> Optional[str]:
    if not date_value or not time_value:
        return None

    if len(date_value) != 6 or len(time_value) < 6:
        return None

    try:
        day = int(date_value[0:2])
        month = int(date_value[2:4])
        year = 2000 + int(date_value[4:6])

        hours = int(time_value[0:2])
        minutes = int(time_value[2:4])
        seconds = float(time_value[4:])
        whole_seconds = int(seconds)
        microseconds = int(round((seconds - whole_seconds) * 1_000_000))

        if microseconds == 1_000_000:
            microseconds = 0
            whole_seconds += 1

        timestamp = datetime(
            year,
            month,
            day,
            hours,
            minutes,
            whole_seconds,
            microseconds,
            tzinfo=timezone.utc,
        )
    except ValueError:
        return None

    return timestamp.isoformat().replace("+00:00", "Z")


def haversine_distance_meters(
    latitude_a: float, longitude_a: float, latitude_b: float, longitude_b: float
) -> float:
    earth_radius_m = 6_371_000.0

    lat_a = math.radians(latitude_a)
    lon_a = math.radians(longitude_a)
    lat_b = math.radians(latitude_b)
    lon_b = math.radians(longitude_b)

    delta_lat = lat_b - lat_a
    delta_lon = lon_b - lon_a

    sin_lat = math.sin(delta_lat / 2.0)
    sin_lon = math.sin(delta_lon / 2.0)
    arc = sin_lat * sin_lat + math.cos(lat_a) * math.cos(lat_b) * sin_lon * sin_lon
    return 2.0 * earth_radius_m * math.asin(math.sqrt(arc))


def parse_gga_fragment(line: str) -> Optional[dict]:
    match = GGA_FRAGMENT_RE.search(line)
    if match is None:
        return None

    lock_quality = parse_int(match.group("quality"))
    return {
        "lock_quality": lock_quality,
        "lock_quality_label": LOCK_QUALITY_LABELS.get(lock_quality),
        "satellite_count_used": parse_int(match.group("satellites")),
        "hdop": parse_float(match.group("hdop")),
        "altitude_m": parse_float(match.group("altitude")),
    }


def parse_gsv_sentence(sentence: str) -> Optional[dict]:
    body = sentence[1:].split("*", 1)[0]
    fields = body.split(",")
    if len(fields) < 4:
        return None

    total_messages = parse_int(fields[1])
    message_number = parse_int(fields[2])
    total_satellites = parse_int(fields[3])
    if total_messages is None or message_number is None:
        return None

    snr_values: list[float] = []
    for index in range(7, len(fields), 4):
        snr = parse_float(fields[index])
        if snr is not None:
            snr_values.append(snr)

    return {
        "talker": fields[0][:2],
        "total_messages": total_messages,
        "message_number": message_number,
        "total_satellites": total_satellites,
        "snr_values": snr_values,
    }


def parse_gsa_sentence(sentence: str) -> Optional[dict]:
    body = sentence[1:].split("*", 1)[0]
    fields = body.split(",")
    if len(fields) < 17:
        return None

    fix_dimension = parse_int(fields[2])
    pdop = parse_float(fields[15])
    hdop = parse_float(fields[16])
    vdop = parse_float(fields[17]) if len(fields) > 17 else None

    return {
        "fix_dimension": fix_dimension,
        "pdop": pdop,
        "hdop": hdop,
        "vdop": vdop,
    }


def rebuild_quality_context(
    quality_context: QualityContext,
    talker_cycles: dict[str, GsvTalkerState],
) -> None:
    snr_values: list[float] = []
    total_satellites = 0

    for talker_state in talker_cycles.values():
        total_satellites += talker_state.total_satellites
        for part_values in talker_state.parts.values():
            snr_values.extend(part_values)

    quality_context.satellite_count_in_view = total_satellites or None
    quality_context.average_snr_dbhz = (
        round(sum(snr_values) / len(snr_values), 3) if snr_values else None
    )
    quality_context.max_snr_dbhz = max(snr_values) if snr_values else None


def timestamp_to_datetime(timestamp: Optional[str]) -> Optional[datetime]:
    if not timestamp:
        return None

    return datetime.fromisoformat(timestamp.replace("Z", "+00:00"))


def wrap_degrees(delta_degrees: float) -> float:
    return ((delta_degrees + 180.0) % 360.0) - 180.0


def add_motion_metrics(points: list[TrackPoint]) -> None:
    if len(points) < 2:
        return

    for previous_point, current_point in zip(points, points[1:]):
        previous_timestamp = timestamp_to_datetime(previous_point.timestamp)
        current_timestamp = timestamp_to_datetime(current_point.timestamp)
        if previous_timestamp is None or current_timestamp is None:
            continue

        delta_seconds = (current_timestamp - previous_timestamp).total_seconds()
        if delta_seconds <= 0.0:
            continue

        if previous_point.speed_mps is not None and current_point.speed_mps is not None:
            longitudinal_accel = (
                current_point.speed_mps - previous_point.speed_mps
            ) / delta_seconds
            current_point.longitudinal_accel_mps2 = longitudinal_accel
        else:
            longitudinal_accel = None

        if (
            previous_point.course_deg is not None
            and current_point.course_deg is not None
            and previous_point.speed_mps is not None
            and current_point.speed_mps is not None
        ):
            delta_heading = wrap_degrees(current_point.course_deg - previous_point.course_deg)
            yaw_rate = math.radians(delta_heading) / delta_seconds
            average_speed = (previous_point.speed_mps + current_point.speed_mps) / 2.0
            current_point.latitudinal_accel_mps2 = average_speed * yaw_rate

        components = [
            value
            for value in (
                current_point.longitudinal_accel_mps2,
                current_point.latitudinal_accel_mps2,
            )
            if value is not None
        ]
        if components:
            current_point.accel_magnitude_mps2 = math.sqrt(
                sum(component * component for component in components)
            )


def parse_rmc_sentence(sentence: str) -> Optional[TrackPoint]:
    body = sentence[1:].split("*", 1)[0]
    fields = body.split(",")
    if len(fields) < 10:
        return None

    sentence_type = fields[0]
    status = fields[2]
    latitude = parse_coordinate(fields[3], fields[4])
    longitude = parse_coordinate(fields[5], fields[6])
    if latitude is None or longitude is None:
        return None

    speed_knots = parse_float(fields[7])
    speed_mps = speed_knots * KNOTS_TO_MPS if speed_knots is not None else None
    course_deg = parse_float(fields[8])
    timestamp = parse_timestamp(fields[9], fields[1])

    if status != "A":
        return None

    return TrackPoint(
        latitude=latitude,
        longitude=longitude,
        timestamp=timestamp,
        speed_knots=speed_knots,
        speed_mps=speed_mps,
        course_deg=course_deg,
        altitude_m=None,
        gps_lock_quality=None,
        gps_lock_quality_label=None,
        satellite_count_used=None,
        satellite_count_in_view=None,
        average_snr_dbhz=None,
        max_snr_dbhz=None,
        hdop=None,
        longitudinal_accel_mps2=None,
        latitudinal_accel_mps2=None,
        accel_magnitude_mps2=None,
        sentence_type=sentence_type,
        raw_sentence=sentence,
    )


def attach_quality_context(point: TrackPoint, quality_context: QualityContext) -> None:
    point.altitude_m = quality_context.altitude_m
    point.gps_lock_quality = quality_context.lock_quality
    point.gps_lock_quality_label = quality_context.lock_quality_label
    point.satellite_count_used = quality_context.satellite_count_used
    point.satellite_count_in_view = quality_context.satellite_count_in_view
    point.average_snr_dbhz = quality_context.average_snr_dbhz
    point.max_snr_dbhz = quality_context.max_snr_dbhz
    point.hdop = quality_context.hdop


def build_geojson(points: list[TrackPoint], source_name: str) -> dict:
    features: list[dict] = []

    if points:
        coordinates = [[point.longitude, point.latitude] for point in points]
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "name": f"{source_name} track",
                    "pointCount": len(points),
                },
                "geometry": {
                    "type": "LineString",
                    "coordinates": coordinates,
                },
            }
        )

        features.append(
            {
                "type": "Feature",
                "properties": {
                    "name": "Start",
                    "timestamp": points[0].timestamp,
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [points[0].longitude, points[0].latitude],
                },
            }
        )

        features.append(
            {
                "type": "Feature",
                "properties": {
                    "name": "End",
                    "timestamp": points[-1].timestamp,
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [points[-1].longitude, points[-1].latitude],
                },
            }
        )

    return {
        "type": "FeatureCollection",
        "features": features,
    }


def build_kml(points: list[TrackPoint], source_name: str) -> str:
    coordinates = "\n".join(
        f"{point.longitude:.8f},{point.latitude:.8f},0" for point in points
    )

    if not coordinates:
        coordinates = ""

    start_placemark = ""
    end_placemark = ""
    if points:
        start_placemark = f"""
    <Placemark>
      <name>Start</name>
      <Point>
        <coordinates>{points[0].longitude:.8f},{points[0].latitude:.8f},0</coordinates>
      </Point>
    </Placemark>"""
        end_placemark = f"""
    <Placemark>
      <name>End</name>
      <Point>
        <coordinates>{points[-1].longitude:.8f},{points[-1].latitude:.8f},0</coordinates>
      </Point>
    </Placemark>"""

    return f"""<?xml version=\"1.0\" encoding=\"UTF-8\"?>
<kml xmlns=\"http://www.opengis.net/kml/2.2\">
  <Document>
    <name>{source_name}</name>
    <Placemark>
      <name>{source_name} track</name>
      <LineString>
        <tessellate>1</tessellate>
        <coordinates>
{coordinates}
        </coordinates>
      </LineString>
    </Placemark>{start_placemark}{end_placemark}
  </Document>
</kml>
"""


def summarize_points(points: list[TrackPoint]) -> tuple[Optional[dict], Optional[dict], Optional[dict], float]:
    if not points:
        return None, None, None, 0.0

    bounds = {
        "minLatitude": min(point.latitude for point in points),
        "maxLatitude": max(point.latitude for point in points),
        "minLongitude": min(point.longitude for point in points),
        "maxLongitude": max(point.longitude for point in points),
    }

    distance_meters = 0.0
    for previous_point, current_point in zip(points, points[1:]):
        distance_meters += haversine_distance_meters(
            previous_point.latitude,
            previous_point.longitude,
            current_point.latitude,
            current_point.longitude,
        )

    return asdict(points[0]), asdict(points[-1]), bounds, distance_meters


def build_report(input_path: Path) -> dict:
    raw_bytes = input_path.read_bytes()
    decoded = raw_bytes.decode("latin-1")
    sentence_counts: Counter[str] = Counter()
    invalid_checksum_count = 0
    points: list[TrackPoint] = []
    quality_context = QualityContext()
    talker_cycles: dict[str, GsvTalkerState] = {}

    for line in decoded.splitlines():
        gga_fragment = parse_gga_fragment(line)
        if gga_fragment is not None:
            quality_context.lock_quality = gga_fragment["lock_quality"]
            quality_context.lock_quality_label = gga_fragment["lock_quality_label"]
            quality_context.satellite_count_used = gga_fragment["satellite_count_used"]
            quality_context.hdop = gga_fragment["hdop"]
            quality_context.altitude_m = gga_fragment["altitude_m"]

        line_sentences, line_invalid_count = iter_valid_sentences(line)
        invalid_checksum_count += line_invalid_count

        for sentence in line_sentences:
            sentence_type = sentence[1:6]
            sentence_counts[sentence_type] += 1

            if sentence_type in GSV_TYPES:
                gsv_data = parse_gsv_sentence(sentence)
                if gsv_data is not None:
                    talker = gsv_data["talker"]
                    total_messages = gsv_data["total_messages"]
                    message_number = gsv_data["message_number"]
                    total_satellites = gsv_data["total_satellites"] or 0
                    talker_state = talker_cycles.get(talker)

                    if (
                        talker_state is None
                        or message_number == 1
                        or talker_state.total_messages != total_messages
                    ):
                        talker_state = GsvTalkerState(
                            total_messages=total_messages,
                            total_satellites=total_satellites,
                            parts={},
                        )
                        talker_cycles[talker] = talker_state

                    talker_state.total_satellites = total_satellites
                    talker_state.parts[message_number] = gsv_data["snr_values"]
                    rebuild_quality_context(quality_context, talker_cycles)
                continue

            if sentence_type in GSA_TYPES:
                gsa_data = parse_gsa_sentence(sentence)
                if gsa_data is not None and gsa_data["hdop"] is not None:
                    quality_context.hdop = gsa_data["hdop"]
                continue

            if sentence_type not in RMC_TYPES:
                continue

            point = parse_rmc_sentence(sentence)
            if point is None:
                continue

            attach_quality_context(point, quality_context)
            points.append(point)

    add_motion_metrics(points)

    first_fix, last_fix, bounds, distance_meters = summarize_points(points)

    return {
        "inputPath": str(input_path.resolve()),
        "sourceName": input_path.name,
        "bytesRead": len(raw_bytes),
        "sentenceCount": sum(sentence_counts.values()),
        "invalidChecksumCount": invalid_checksum_count,
        "sentenceTypeCounts": dict(sorted(sentence_counts.items())),
        "fixCount": len(points),
        "firstFix": first_fix,
        "lastFix": last_fix,
        "bounds": bounds,
        "distanceMeters": round(distance_meters, 3),
        "distanceKilometers": round(distance_meters / 1000.0, 3),
        "points": [asdict(point) for point in points],
        "geojson": build_geojson(points, input_path.name),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Parse mixed binary plus NMEA GNSS logs into map-ready track data."
    )
    parser.add_argument("input", help="Path to the log file to parse.")
    parser.add_argument(
        "--geojson-output",
        help="Optional path to write a GeoJSON export.",
    )
    parser.add_argument(
        "--kml-output",
        help="Optional path to write a KML export.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = Path(args.input)
    if not input_path.exists():
        raise ParserError(f"Input file does not exist: {input_path}")
    if not input_path.is_file():
        raise ParserError(f"Input path is not a file: {input_path}")

    report = build_report(input_path)

    if args.geojson_output:
        geojson_path = Path(args.geojson_output)
        geojson_path.write_text(json.dumps(report["geojson"], indent=2), encoding="utf-8")

    if args.kml_output:
        kml_path = Path(args.kml_output)
        points = [TrackPoint(**point) for point in report["points"]]
        kml_path.write_text(build_kml(points, input_path.name), encoding="utf-8")

    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ParserError as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
