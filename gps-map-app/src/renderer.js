const defaultCenter = [34.0684, -118.4435];
const defaultZoom = 13;
const KPH_TO_MPH = 0.621371;
const KNOTS_TO_MPH = 1.150779;
const MPS_TO_MPH = 2.236936;
const ACCEL_COLOR_PERCENTILE = 0.95;
const LAP_DETECTION_RADIUS_METERS = 1;
const DIVIDER_SNAP_MAX_DISTANCE_METERS = 25;
const speedHeatColors = ['#1e3a8a', '#2563eb', '#0ea5a4', '#4ade80', '#facc15', '#fb923c', '#dc2626'];
const qualityColors = ['#7f1d1d', '#b91c1c', '#ea580c', '#eab308', '#84cc16', '#16a34a'];
const sectorDividerColors = ['#f16839', '#0a8f7b', '#2563eb', '#eab308', '#ef4444', '#7c3aed', '#0f766e', '#fb923c'];
const visualizationOrder = ['speedAccel', 'lockQuality', 'snr', 'satelliteCount'];

const TRACK_SOURCE_ID = 'track-source';
const BARS_SOURCE_ID = 'bars-source';
const HITS_SOURCE_ID = 'hits-source';
const MARKERS_SOURCE_ID = 'markers-source';
const SELECTED_SOURCE_ID = 'selected-source';
const LAP_CROSSINGS_SOURCE_ID = 'lap-crossings-source';
const TERRAIN_SOURCE_ID = 'terrain-source';

const TRACK_GLOW_LAYER_ID = 'track-glow-layer';
const TRACK_LINE_LAYER_ID = 'track-line-layer';
const BARS_LAYER_ID = 'bars-layer';
const SPEED_HEATMAP_LAYER_ID = 'speed-heatmap-layer';
const HITS_LAYER_ID = 'hits-layer';
const MARKERS_LAYER_ID = 'markers-layer';
const SELECTED_LAYER_ID = 'selected-layer';
const LAP_CROSSINGS_LAYER_ID = 'lap-crossings-layer';

const MAP_3D_PITCH = 62;
const MAP_3D_BEARING = -24;
const MAP_2D_PITCH = 0;
const MAP_2D_BEARING = 0;

const elements = {
  openLogButton: document.getElementById('open-log-button'),
  selectedFile: document.getElementById('selected-file'),
  exportGeoJsonButton: document.getElementById('export-geojson-button'),
  exportKmlButton: document.getElementById('export-kml-button'),
  analysisTabButtons: Array.from(document.querySelectorAll('.sidebar-tab')),
  analysisTabMap: document.getElementById('analysis-tab-map'),
  analysisTabLap: document.getElementById('analysis-tab-lap'),
  sidebarPaneMap: document.getElementById('sidebar-pane-map'),
  sidebarPaneLap: document.getElementById('sidebar-pane-lap'),
  fixCount: document.getElementById('fix-count'),
  distanceKm: document.getElementById('distance-km'),
  sentenceCount: document.getElementById('sentence-count'),
  checksumCount: document.getElementById('checksum-count'),
  timeSpan: document.getElementById('time-span'),
  startFix: document.getElementById('start-fix'),
  endFix: document.getElementById('end-fix'),
  trackBounds: document.getElementById('track-bounds'),
  topSpeed: document.getElementById('top-speed'),
  sentenceCounts: document.getElementById('sentence-counts'),
  rmcShare: document.getElementById('rmc-share'),
  statusMessage: document.getElementById('status-message'),
  mapTitle: document.getElementById('map-title'),
  mapBadge: document.getElementById('map-badge'),
  mapOverlayMessage: document.getElementById('map-overlay-message'),
  speedLegend: document.getElementById('speed-legend'),
  speedLegendCaption: document.getElementById('speed-legend-caption'),
  speedLegendPeak: document.getElementById('speed-legend-peak'),
  speedLegendMin: document.getElementById('speed-legend-min'),
  speedLegendMid: document.getElementById('speed-legend-mid'),
  speedLegendMax: document.getElementById('speed-legend-max'),
  barSizeRange: document.getElementById('bar-size-range'),
  barSizeValue: document.getElementById('bar-size-value'),
  lapSectorCount: document.getElementById('lap-sector-count'),
  lapDividerList: document.getElementById('lap-divider-list'),
  lapAnalysisChip: document.getElementById('lap-analysis-chip'),
  lapSectorSummary: document.getElementById('lap-sector-summary'),
  lapSectorTable: document.getElementById('lap-sector-table'),
  visualizationInputs: Array.from(document.querySelectorAll('.visualization-option input')),
  pointDetailOverlay: document.getElementById('point-detail-overlay'),
  pointDetailClose: document.getElementById('point-detail-close'),
  pointDetailTitle: document.getElementById('point-detail-title'),
  pointDetailTime: document.getElementById('point-detail-time'),
  pointDetailSubtitle: document.getElementById('point-detail-subtitle'),
  pointDetailSummary: document.getElementById('point-detail-summary'),
  pointRadarChart: document.getElementById('point-radar-chart'),
  pointRadarValues: document.getElementById('point-radar-values'),
  // PQTM / RTCM elements
  pqtmRtkChip: document.getElementById('pqtm-rtk-chip'),
  pqtmVehattCount: document.getElementById('pqtm-vehatt-count'),
  pqtmTarCount: document.getElementById('pqtm-tar-count'),
  pqtmTarFixed: document.getElementById('pqtm-tar-fixed'),
  pqtmTarFloat: document.getElementById('pqtm-tar-float'),
  pqtmHeading: document.getElementById('pqtm-heading'),
  pqtmRollPitch: document.getElementById('pqtm-roll-pitch'),
  pqtmDist: document.getElementById('pqtm-dist'),
  rtcmChip: document.getElementById('rtcm-chip'),
  rtcmFrameCount: document.getElementById('rtcm-frame-count'),
  rtcmCrcErrors: document.getElementById('rtcm-crc-errors'),
  rtcmBytes: document.getElementById('rtcm-bytes'),
  rtcmMsgTypes: document.getElementById('rtcm-msg-types'),
};

const overlayDefinitions = {
  speedAccel: {
    label: 'Speed / accel',
    heightValue: (point) => speedMph(point),
    heightRangeKey: 'speedMaxMph',
    colorValue: (point) => absoluteOrNull(point.accel_magnitude_mps2),
    colorRangeKey: 'accelColorMaxAbs',
    palette: speedHeatColors,
    maxHeightMeters: 150,
  },
  lockQuality: {
    label: 'GPS lock quality',
    heightValue: (point) => point.gps_lock_quality,
    heightRangeKey: 'qualityMax',
    colorValue: (point) => point.gps_lock_quality,
    colorRangeKey: 'qualityMax',
    palette: qualityColors,
    maxHeightMeters: 72,
  },
  snr: {
    label: 'Average SNR',
    heightValue: (point) => point.average_snr_dbhz,
    heightRangeKey: 'snrMax',
    colorValue: (point) => point.average_snr_dbhz,
    colorRangeKey: 'snrMax',
    palette: speedHeatColors,
    maxHeightMeters: 92,
  },
  satelliteCount: {
    label: 'Satellite count',
    heightValue: (point) => satelliteCountValue(point),
    heightRangeKey: 'satelliteMax',
    colorValue: (point) => satelliteCountValue(point),
    colorRangeKey: 'satelliteMax',
    palette: speedHeatColors,
    maxHeightMeters: 84,
  },
};

const state = {
  busy: false,
  activeTab: 'map',
  mapReady: false,
  currentFilePath: null,
  currentReport: null,
  selectedPointIndex: null,
  barSizeScale: 1,
  lapAnalysis: {
    sectorCount: 3,
    activeDividerIndex: 0,
    detectionRadiusMeters: LAP_DETECTION_RADIUS_METERS,
    dividerPins: Array.from({ length: 3 }, () => null),
    dividerMarkers: [],
    trackSamples: [],
    analysis: null,
  },
  metricRanges: null,
  activeVisualizations: new Set(
    elements.visualizationInputs.filter((input) => input.checked).map((input) => input.value),
  ),
};

const map = new maplibregl.Map({
  container: 'map',
  style: buildMapStyle(),
  center: [defaultCenter[1], defaultCenter[0]],
  zoom: defaultZoom,
  pitch: MAP_3D_PITCH,
  bearing: MAP_3D_BEARING,
  antialias: true,
  maxPitch: 80,
});

map.addControl(new maplibregl.NavigationControl({ visualizePitch: true }), 'bottom-right');
map.on('load', handleMapLoad);
map.on('click', handleMapClick);

function buildMapStyle() {
  return {
    version: 8,
    sources: {
      'osm-raster': {
        type: 'raster',
        tiles: ['https://tile.openstreetmap.org/{z}/{x}/{y}.png'],
        tileSize: 256,
        attribution: '&copy; OpenStreetMap contributors',
      },
      [TERRAIN_SOURCE_ID]: {
        type: 'raster-dem',
        url: 'https://demotiles.maplibre.org/terrain-tiles/tiles.json',
        tileSize: 256,
      },
    },
    layers: [
      {
        id: 'osm-raster-layer',
        type: 'raster',
        source: 'osm-raster',
        minzoom: 0,
        maxzoom: 19,
      },
    ],
    terrain: {
      source: TERRAIN_SOURCE_ID,
      exaggeration: 1.18,
    },
  };
}

function featureCollection(features = []) {
  return {
    type: 'FeatureCollection',
    features,
  };
}

function finiteNumber(value) {
  return typeof value === 'number' && Number.isFinite(value);
}

function absoluteOrNull(value) {
  return finiteNumber(value) ? Math.abs(value) : null;
}

function firstDefined(...values) {
  for (const value of values) {
    if (value !== null && value !== undefined) {
      return value;
    }
  }
  return null;
}

function shortFileName(path) {
  if (!path) {
    return 'No file selected';
  }
  const parts = String(path).split(/[/\\]/);
  return parts[parts.length - 1] || String(path);
}

function escapeHtml(value) {
  return String(value ?? '')
    .replaceAll('&', '&amp;')
    .replaceAll('<', '&lt;')
    .replaceAll('>', '&gt;')
    .replaceAll('"', '&quot;')
    .replaceAll("'", '&#39;');
}

function formatNumber(value, digits = 0) {
  if (!finiteNumber(value)) {
    return '--';
  }
  return Number(value).toLocaleString(undefined, {
    minimumFractionDigits: digits,
    maximumFractionDigits: digits,
  });
}

function formatTimestamp(timestamp) {
  if (!timestamp) {
    return 'Unknown time';
  }
  const parsed = new Date(timestamp);
  if (Number.isNaN(parsed.getTime())) {
    return String(timestamp);
  }
  return parsed.toLocaleString();
}

function formatDuration(startTimestamp, endTimestamp) {
  if (!startTimestamp || !endTimestamp) {
    return 'No fixes yet';
  }
  const start = new Date(startTimestamp);
  const end = new Date(endTimestamp);
  const durationMs = end.getTime() - start.getTime();
  if (!Number.isFinite(durationMs) || durationMs < 0) {
    return 'No fixes yet';
  }
  const totalSeconds = Math.round(durationMs / 1000);
  const hours = Math.floor(totalSeconds / 3600);
  const minutes = Math.floor((totalSeconds % 3600) / 60);
  const seconds = totalSeconds % 60;
  if (hours > 0) {
    return `${hours}h ${minutes}m ${seconds}s`;
  }
  if (minutes > 0) {
    return `${minutes}m ${seconds}s`;
  }
  return `${seconds}s`;
}

function formatCoordinatePair(point) {
  if (!point || !finiteNumber(point.latitude) || !finiteNumber(point.longitude)) {
    return '--';
  }
  return `${formatNumber(point.latitude, 6)}, ${formatNumber(point.longitude, 6)}`;
}

function formatSeconds(value, digits = 3) {
  if (!finiteNumber(value)) {
    return '--';
  }
  return `${formatNumber(value, digits)} s`;
}

function formatDistanceMeters(value) {
  if (!finiteNumber(value)) {
    return '--';
  }
  if (value >= 1000) {
    return `${formatNumber(value / 1000, 3)} km`;
  }
  return `${formatNumber(value, 1)} m`;
}

function timestampToDate(timestamp) {
  if (!timestamp) {
    return null;
  }

  const parsed = new Date(timestamp);
  return Number.isNaN(parsed.getTime()) ? null : parsed;
}

function haversineDistanceMeters(latitudeA, longitudeA, latitudeB, longitudeB) {
  const earthRadiusMeters = 6371000;
  const latitudeARadians = latitudeA * (Math.PI / 180);
  const latitudeBRadians = latitudeB * (Math.PI / 180);
  const deltaLatitudeRadians = (latitudeB - latitudeA) * (Math.PI / 180);
  const deltaLongitudeRadians = (longitudeB - longitudeA) * (Math.PI / 180);

  const haversine = (
    (Math.sin(deltaLatitudeRadians / 2) ** 2)
    + (Math.cos(latitudeARadians) * Math.cos(latitudeBRadians) * (Math.sin(deltaLongitudeRadians / 2) ** 2))
  );

  return 2 * earthRadiusMeters * Math.atan2(Math.sqrt(haversine), Math.sqrt(1 - haversine));
}

function buildTrackSamples(points) {
  const samples = [];
  let cumulativeDistanceMeters = 0;
  const startDate = timestampToDate(points[0]?.timestamp);

  for (const [pointIndex, point] of points.entries()) {
    if (pointIndex > 0) {
      const previousPoint = points[pointIndex - 1];
      cumulativeDistanceMeters += haversineDistanceMeters(
        previousPoint.latitude,
        previousPoint.longitude,
        point.latitude,
        point.longitude,
      );
    }

    const pointDate = timestampToDate(point.timestamp);
    samples.push({
      pointIndex,
      latitude: point.latitude,
      longitude: point.longitude,
      timestamp: point.timestamp,
      elapsedSeconds: startDate && pointDate
        ? (pointDate.getTime() - startDate.getTime()) / 1000
        : null,
      cumulativeDistanceMeters,
    });
  }

  return samples;
}

function speedMph(point) {
  if (finiteNumber(point.speed_kph)) {
    return point.speed_kph * KPH_TO_MPH;
  }
  if (finiteNumber(point.speed_knots)) {
    return point.speed_knots * KNOTS_TO_MPH;
  }
  if (finiteNumber(point.speed_mps)) {
    return point.speed_mps * MPS_TO_MPH;
  }
  return null;
}

function satelliteCountValue(point) {
  return firstDefined(point.satellite_count_used, point.satellite_count_in_view);
}

function lockQualityText(point) {
  if (point.gps_lock_quality_label) {
    if (finiteNumber(point.gps_lock_quality)) {
      return `${point.gps_lock_quality_label} (${formatNumber(point.gps_lock_quality)})`;
    }
    return point.gps_lock_quality_label;
  }
  if (finiteNumber(point.gps_lock_quality)) {
    return formatNumber(point.gps_lock_quality);
  }
  return '--';
}

function paletteColor(normalized, palette) {
  if (!palette.length) {
    return '#ffffff';
  }
  if (!finiteNumber(normalized)) {
    return palette[0];
  }
  const clamped = Math.max(0, Math.min(0.999999, normalized));
  const index = Math.min(palette.length - 1, Math.floor(clamped * palette.length));
  return palette[index];
}

function normalizeAgainst(value, maximum) {
  if (!finiteNumber(value) || !finiteNumber(maximum) || maximum <= 0) {
    return null;
  }
  return Math.max(0, Math.min(1, value / maximum));
}

function percentileValue(values, percentile) {
  if (!values.length) {
    return 0;
  }

  const sorted = [...values].sort((left, right) => left - right);
  const index = Math.min(
    sorted.length - 1,
    Math.max(0, Math.floor((sorted.length - 1) * percentile)),
  );
  return sorted[index];
}

function activeOverlayIds() {
  return visualizationOrder.filter((id) => state.activeVisualizations.has(id));
}

function usingVelocityFallback() {
  return activeOverlayIds().length === 0;
}

function overlaySummaryText() {
  const labels = activeOverlayIds().map((id) => overlayDefinitions[id].label);
  return labels.length ? labels.join(', ') : 'velocity heatmap fallback';
}

function setBusy(nextBusy) {
  state.busy = nextBusy;
  elements.openLogButton.disabled = nextBusy;
  const exportEnabled = Boolean(state.currentReport) && !nextBusy;
  elements.exportGeoJsonButton.disabled = !exportEnabled;
  elements.exportKmlButton.disabled = !exportEnabled;
}

function setStatus(message, badgeText = 'Ready') {
  elements.statusMessage.textContent = message;
  elements.mapBadge.textContent = badgeText;
}

function renderBarSizeLabel() {
  elements.barSizeValue.textContent = `${state.barSizeScale.toFixed(1)}x`;
}

function hideSpeedLegend() {
  elements.speedLegend.hidden = true;
}

function showSpeedLegend(metricRanges, caption = 'Speed / accel') {
  if (!metricRanges || !finiteNumber(metricRanges.speedMaxMph) || metricRanges.speedMaxMph <= 0) {
    hideSpeedLegend();
    return;
  }
  elements.speedLegend.hidden = false;
  elements.speedLegendCaption.textContent = caption;
  elements.speedLegendPeak.textContent = `Peak ${formatNumber(metricRanges.speedMaxMph, 1)} mph`;
  elements.speedLegendMin.textContent = '0 mph';
  elements.speedLegendMid.textContent = `${formatNumber(metricRanges.speedMaxMph / 2, 1)} mph`;
  elements.speedLegendMax.textContent = `${formatNumber(metricRanges.speedMaxMph, 1)} mph`;
}

function cameraPoseForCurrentMode() {
  if (usingVelocityFallback()) {
    return {
      pitch: MAP_2D_PITCH,
      bearing: MAP_2D_BEARING,
      terrain: null,
    };
  }

  return {
    pitch: MAP_3D_PITCH,
    bearing: MAP_3D_BEARING,
    terrain: {
      source: TERRAIN_SOURCE_ID,
      exaggeration: 1.18,
    },
  };
}

function applyMapPresentation() {
  if (!state.mapReady) {
    return;
  }

  const pose = cameraPoseForCurrentMode();
  if (typeof map.setTerrain === 'function') {
    map.setTerrain(pose.terrain);
  }

  if (map.getLayer(SPEED_HEATMAP_LAYER_ID)) {
    map.setLayoutProperty(
      SPEED_HEATMAP_LAYER_ID,
      'visibility',
      usingVelocityFallback() ? 'visible' : 'none',
    );
  }
}

function setSourceData(sourceId, data) {
  if (!state.mapReady) {
    return;
  }
  const source = map.getSource(sourceId);
  if (source) {
    source.setData(data);
  }
}

function dividerColor(index) {
  return sectorDividerColors[index % sectorDividerColors.length];
}

function countPinnedDividers() {
  return state.lapAnalysis.dividerPins.filter(Boolean).length;
}

function nextUnpinnedDividerIndex(currentIndex) {
  const { dividerPins } = state.lapAnalysis;

  for (let offset = 1; offset <= dividerPins.length; offset += 1) {
    const candidateIndex = (currentIndex + offset) % dividerPins.length;
    if (!dividerPins[candidateIndex]) {
      return candidateIndex;
    }
  }

  return null;
}

function resizeDividerPins(nextCount) {
  const resizedPins = state.lapAnalysis.dividerPins.slice(0, nextCount);
  while (resizedPins.length < nextCount) {
    resizedPins.push(null);
  }

  state.lapAnalysis.sectorCount = nextCount;
  state.lapAnalysis.dividerPins = resizedPins;

  if (
    Number.isInteger(state.lapAnalysis.activeDividerIndex)
    && state.lapAnalysis.activeDividerIndex >= nextCount
  ) {
    state.lapAnalysis.activeDividerIndex = nextCount - 1;
  }

  if (!Number.isInteger(state.lapAnalysis.activeDividerIndex)) {
    state.lapAnalysis.activeDividerIndex = nextCount ? 0 : null;
  }
}

function buildLapCrossingsForPin(pin, dividerIndex, radiusMeters) {
  const crossings = [];
  let crossingRun = [];

  const flushCrossingRun = () => {
    if (!crossingRun.length) {
      return;
    }

    const nearestSample = crossingRun.reduce((bestSample, candidateSample) => (
      candidateSample.distanceMeters < bestSample.distanceMeters ? candidateSample : bestSample
    ));

    crossings.push({
      dividerIndex,
      pointIndex: nearestSample.pointIndex,
      latitude: nearestSample.latitude,
      longitude: nearestSample.longitude,
      timestamp: nearestSample.timestamp,
      elapsedSeconds: nearestSample.elapsedSeconds,
      cumulativeDistanceMeters: nearestSample.cumulativeDistanceMeters,
      distanceMeters: nearestSample.distanceMeters,
    });

    crossingRun = [];
  };

  for (const sample of state.lapAnalysis.trackSamples) {
    const distanceMeters = haversineDistanceMeters(
      pin.latitude,
      pin.longitude,
      sample.latitude,
      sample.longitude,
    );

    if (distanceMeters <= radiusMeters) {
      crossingRun.push({
        ...sample,
        distanceMeters,
      });
    } else {
      flushCrossingRun();
    }
  }

  flushCrossingRun();
  return crossings;
}

function analyzeLapTrack() {
  const dividerSummaries = state.lapAnalysis.dividerPins.map((pin, dividerIndex) => {
    if (!pin) {
      return {
        dividerIndex,
        pin: null,
        crossings: [],
      };
    }

    return {
      dividerIndex,
      pin,
      crossings: buildLapCrossingsForPin(
        pin,
        dividerIndex,
        state.lapAnalysis.detectionRadiusMeters,
      ),
    };
  });

  const orderedDividerIndexes = dividerSummaries
    .filter((summary) => summary.pin && summary.crossings.length)
    .sort((leftSummary, rightSummary) => leftSummary.crossings[0].pointIndex - rightSummary.crossings[0].pointIndex)
    .map((summary) => summary.dividerIndex);

  const crossings = dividerSummaries
    .flatMap((summary) => summary.crossings)
    .sort((leftCrossing, rightCrossing) => leftCrossing.pointIndex - rightCrossing.pointIndex);

  const pinnedCount = countPinnedDividers();
  const ready = (
    pinnedCount === state.lapAnalysis.sectorCount
    && orderedDividerIndexes.length === state.lapAnalysis.sectorCount
  );

  const sectorStats = [];
  let totalSegments = 0;

  if (ready) {
    const dividerOrder = new Map(
      orderedDividerIndexes.map((dividerIndex, orderIndex) => [dividerIndex, orderIndex]),
    );

    const sectorBuilders = orderedDividerIndexes.map((dividerIndex, orderIndex) => ({
      sectorNumber: orderIndex + 1,
      startDividerIndex: dividerIndex,
      endDividerIndex: orderedDividerIndexes[(orderIndex + 1) % orderedDividerIndexes.length],
      segments: [],
    }));

    for (let crossingIndex = 0; crossingIndex < crossings.length - 1; crossingIndex += 1) {
      const startCrossing = crossings[crossingIndex];
      const endCrossing = crossings[crossingIndex + 1];
      const startOrder = dividerOrder.get(startCrossing.dividerIndex);
      const endOrder = dividerOrder.get(endCrossing.dividerIndex);

      if (!Number.isInteger(startOrder) || !Number.isInteger(endOrder)) {
        continue;
      }

      const expectedNextOrder = orderedDividerIndexes.length === 1
        ? startOrder
        : ((startOrder + 1) % orderedDividerIndexes.length);
      if (endOrder !== expectedNextOrder) {
        continue;
      }

      sectorBuilders[startOrder].segments.push({
        startPointIndex: startCrossing.pointIndex,
        endPointIndex: endCrossing.pointIndex,
        timeSeconds: (
          finiteNumber(startCrossing.elapsedSeconds)
          && finiteNumber(endCrossing.elapsedSeconds)
        )
          ? endCrossing.elapsedSeconds - startCrossing.elapsedSeconds
          : null,
        distanceMeters: (
          finiteNumber(startCrossing.cumulativeDistanceMeters)
          && finiteNumber(endCrossing.cumulativeDistanceMeters)
        )
          ? endCrossing.cumulativeDistanceMeters - startCrossing.cumulativeDistanceMeters
          : null,
      });
    }

    for (const sectorBuilder of sectorBuilders) {
      const validTimes = sectorBuilder.segments
        .map((segment) => segment.timeSeconds)
        .filter(finiteNumber);
      const validDistances = sectorBuilder.segments
        .map((segment) => segment.distanceMeters)
        .filter(finiteNumber);
      totalSegments += sectorBuilder.segments.length;

      sectorStats.push({
        sectorNumber: sectorBuilder.sectorNumber,
        startDividerIndex: sectorBuilder.startDividerIndex,
        endDividerIndex: sectorBuilder.endDividerIndex,
        segmentsCount: sectorBuilder.segments.length,
        bestTimeSeconds: validTimes.length ? Math.min(...validTimes) : null,
        averageTimeSeconds: validTimes.length
          ? validTimes.reduce((sum, value) => sum + value, 0) / validTimes.length
          : null,
        latestTimeSeconds: validTimes.length ? validTimes[validTimes.length - 1] : null,
        averageDistanceMeters: validDistances.length
          ? validDistances.reduce((sum, value) => sum + value, 0) / validDistances.length
          : null,
      });
    }
  }

  return {
    dividerSummaries,
    orderedDividerIndexes,
    crossings,
    pinnedCount,
    ready,
    sectorStats,
    totalSegments,
    lapCount: ready && state.lapAnalysis.sectorCount > 0
      ? Math.floor(totalSegments / state.lapAnalysis.sectorCount)
      : 0,
  };
}

function buildLapCrossingFeatureCollection() {
  if (
    state.activeTab !== 'lapAnalysis'
    || !state.currentReport
    || !state.lapAnalysis.analysis?.crossings.length
  ) {
    return featureCollection();
  }

  return featureCollection(
    state.lapAnalysis.analysis.crossings.map((crossing) => ({
      type: 'Feature',
      properties: {
        dividerIndex: crossing.dividerIndex,
        color: dividerColor(crossing.dividerIndex),
      },
      geometry: {
        type: 'Point',
        coordinates: [crossing.longitude, crossing.latitude],
      },
    })),
  );
}

function removeDividerMarkers() {
  for (const marker of state.lapAnalysis.dividerMarkers) {
    marker.remove();
  }
  state.lapAnalysis.dividerMarkers = [];
}

function updateDividerMarkers() {
  removeDividerMarkers();

  if (!state.mapReady || !state.currentReport || state.activeTab !== 'lapAnalysis') {
    return;
  }

  state.lapAnalysis.dividerPins.forEach((pin, dividerIndex) => {
    if (!pin) {
      return;
    }

    const markerElement = document.createElement('button');
    markerElement.type = 'button';
    markerElement.className = 'sector-pin-marker';
    markerElement.textContent = `${dividerIndex + 1}`;
    markerElement.style.setProperty('--pin-color', dividerColor(dividerIndex));
    if (state.lapAnalysis.activeDividerIndex === dividerIndex) {
      markerElement.classList.add('is-active');
    }
    markerElement.addEventListener('click', (event) => {
      event.preventDefault();
      event.stopPropagation();
      state.lapAnalysis.activeDividerIndex = dividerIndex;
      renderLapAnalysis();
      updateDividerMarkers();
    });

    const marker = new maplibregl.Marker({
      element: markerElement,
      draggable: true,
      anchor: 'center',
    })
      .setLngLat([pin.longitude, pin.latitude])
      .addTo(map);

    marker.on('dragend', () => {
      const markerPosition = marker.getLngLat();
      setDividerPin(
        dividerIndex,
        {
          latitude: markerPosition.lat,
          longitude: markerPosition.lng,
        },
        {
          advance: false,
          statusMessage: `Moved divider ${dividerIndex + 1}.`,
        },
      );
    });

    state.lapAnalysis.dividerMarkers.push(marker);
  });
}

function renderTabState() {
  const showingMapTab = state.activeTab === 'map';

  elements.sidebarPaneMap.hidden = !showingMapTab;
  elements.sidebarPaneLap.hidden = showingMapTab;
  elements.analysisTabMap.classList.toggle('is-active', showingMapTab);
  elements.analysisTabLap.classList.toggle('is-active', !showingMapTab);
  elements.analysisTabMap.setAttribute('aria-selected', `${showingMapTab}`);
  elements.analysisTabLap.setAttribute('aria-selected', `${!showingMapTab}`);

  if (!showingMapTab) {
    closePointDetail();
  }
}

function renderLapDividerList() {
  const analysis = state.lapAnalysis.analysis;

  elements.lapDividerList.innerHTML = state.lapAnalysis.dividerPins
    .map((pin, dividerIndex) => {
      const dividerSummary = analysis?.dividerSummaries?.[dividerIndex];
      const crossingsCount = dividerSummary?.crossings.length ?? 0;
      const isArmed = state.lapAnalysis.activeDividerIndex === dividerIndex;
      const placementText = pin
        ? `${formatNumber(pin.latitude, 6)}, ${formatNumber(pin.longitude, 6)}`
        : 'Not pinned yet.';
      const statusText = pin
        ? `${placementText} • ${formatNumber(crossingsCount)} detected crossings`
        : 'Arm this divider and click the map near the track to place it.';
      const actionText = isArmed
        ? 'Click map to place'
        : (pin ? 'Reposition on map' : 'Place on map');

      return `
        <article class="lap-divider-item${isArmed ? ' is-armed' : ''}" style="--divider-color: ${dividerColor(dividerIndex)};">
          <div class="lap-divider-row">
            <div class="lap-divider-title">
              <span class="lap-divider-swatch"></span>
              <strong>Divider ${dividerIndex + 1}</strong>
            </div>
            <span class="chip muted">${formatNumber(crossingsCount)} hits</span>
          </div>
          <p class="lap-divider-meta">${escapeHtml(statusText)}</p>
          <div class="lap-divider-actions">
            <button
              type="button"
              class="lap-divider-button${isArmed ? ' is-armed' : ''}"
              data-action="arm-divider"
              data-divider-index="${dividerIndex}"
              ${state.currentReport ? '' : 'disabled'}
            >
              ${escapeHtml(actionText)}
            </button>
            <button
              type="button"
              class="lap-divider-button"
              data-action="clear-divider"
              data-divider-index="${dividerIndex}"
              ${pin ? '' : 'disabled'}
            >
              Clear
            </button>
          </div>
        </article>
      `;
    })
    .join('');
}

function renderLapAnalysisResults() {
  const analysis = state.lapAnalysis.analysis;
  const pinnedCount = analysis?.pinnedCount ?? countPinnedDividers();
  elements.lapSectorCount.value = `${state.lapAnalysis.sectorCount}`;
  elements.lapSectorCount.disabled = !state.currentReport;

  if (!state.currentReport) {
    elements.lapAnalysisChip.textContent = 'No data';
    elements.lapSectorSummary.innerHTML = `
      <article class="lap-empty-state">
        Load a parsed GNSS track to start pinning sector dividers.
      </article>
    `;
    elements.lapSectorTable.innerHTML = '';
    return;
  }

  const summaryCards = [
    {
      label: 'Pinned dividers',
      value: `${pinnedCount}/${state.lapAnalysis.sectorCount}`,
    },
    {
      label: 'Detected crossings',
      value: formatNumber(analysis?.crossings.length ?? 0),
    },
    {
      label: 'Ready sectors',
      value: analysis?.ready ? formatNumber(analysis.sectorStats.length) : '--',
    },
    {
      label: 'Estimated laps',
      value: analysis?.ready ? formatNumber(analysis.lapCount) : '--',
    },
  ];

  elements.lapSectorSummary.innerHTML = summaryCards
    .map(
      (card) => `
        <article class="lap-summary-card">
          <span>${escapeHtml(card.label)}</span>
          <strong>${escapeHtml(card.value)}</strong>
        </article>
      `,
    )
    .join('');

  if (!analysis) {
    elements.lapAnalysisChip.textContent = 'No data';
    elements.lapSectorTable.innerHTML = '';
    return;
  }

  const missingPins = state.lapAnalysis.dividerPins
    .map((pin, dividerIndex) => (pin ? null : dividerIndex + 1))
    .filter(Boolean);
  const undetectedPins = analysis.dividerSummaries
    .filter((summary) => summary.pin && summary.crossings.length === 0)
    .map((summary) => summary.dividerIndex + 1);

  if (!analysis.ready) {
    elements.lapAnalysisChip.textContent = `${pinnedCount}/${state.lapAnalysis.sectorCount} pinned`;

    let message = 'Pin each divider to start detecting sectors.';
    if (missingPins.length) {
      message = `Still missing divider pins: ${missingPins.join(', ')}.`;
    } else if (undetectedPins.length) {
      message = `Pinned dividers ${undetectedPins.join(', ')} do not yet capture any fixes within +/- 1 m. Drag them onto the line or click closer to the track.`;
    }

    elements.lapSectorTable.innerHTML = `
      <article class="lap-empty-state">
        ${escapeHtml(message)}
      </article>
    `;
    return;
  }

  elements.lapAnalysisChip.textContent = `${formatNumber(analysis.lapCount)} laps / ${formatNumber(analysis.totalSegments)} segments`;
  elements.lapSectorTable.innerHTML = analysis.sectorStats
    .map((sector) => {
      const routeText = state.lapAnalysis.sectorCount === 1
        ? 'Repeated crossings on Divider 1'
        : `Divider ${sector.startDividerIndex + 1} -> Divider ${sector.endDividerIndex + 1}`;

      return `
        <article class="lap-sector-card" style="--divider-color: ${dividerColor(sector.startDividerIndex)};">
          <div class="lap-sector-card-header">
            <div class="lap-sector-title">
              <span class="lap-sector-swatch"></span>
              <strong>Sector ${sector.sectorNumber}</strong>
            </div>
            <span class="chip muted">${formatNumber(sector.segmentsCount)} runs</span>
          </div>
          <p class="lap-sector-route">${escapeHtml(routeText)}</p>
          <div class="lap-sector-metrics">
            <div class="lap-sector-metric">
              <strong>${escapeHtml(formatSeconds(sector.bestTimeSeconds))}</strong>
              <span>Best</span>
            </div>
            <div class="lap-sector-metric">
              <strong>${escapeHtml(formatSeconds(sector.averageTimeSeconds))}</strong>
              <span>Average</span>
            </div>
            <div class="lap-sector-metric">
              <strong>${escapeHtml(formatSeconds(sector.latestTimeSeconds))}</strong>
              <span>Latest</span>
            </div>
            <div class="lap-sector-metric">
              <strong>${escapeHtml(formatDistanceMeters(sector.averageDistanceMeters))}</strong>
              <span>Avg distance</span>
            </div>
          </div>
        </article>
      `;
    })
    .join('');
}

function renderLapAnalysis() {
  renderTabState();
  renderLapDividerList();
  renderLapAnalysisResults();
}

function refreshLapCrossingsSource() {
  const crossingData = buildLapCrossingFeatureCollection();
  setSourceData(LAP_CROSSINGS_SOURCE_ID, crossingData);

  if (state.mapReady && map.getLayer(LAP_CROSSINGS_LAYER_ID)) {
    map.setLayoutProperty(
      LAP_CROSSINGS_LAYER_ID,
      'visibility',
      state.activeTab === 'lapAnalysis' && crossingData.features.length ? 'visible' : 'none',
    );
  }
}

function refreshLapAnalysis() {
  state.lapAnalysis.analysis = analyzeLapTrack();
  renderLapAnalysis();
  refreshLapCrossingsSource();
  updateDividerMarkers();
}

function resetLapAnalysisForReport(report) {
  state.lapAnalysis.trackSamples = buildTrackSamples(report.points);
  state.lapAnalysis.dividerPins = Array.from(
    { length: state.lapAnalysis.sectorCount },
    () => null,
  );
  state.lapAnalysis.activeDividerIndex = state.lapAnalysis.sectorCount ? 0 : null;
  state.lapAnalysis.analysis = null;
  refreshLapAnalysis();
}

function setDividerPin(dividerIndex, pin, options = {}) {
  state.lapAnalysis.dividerPins[dividerIndex] = {
    latitude: pin.latitude,
    longitude: pin.longitude,
    pointIndex: Number.isInteger(pin.pointIndex) ? pin.pointIndex : null,
  };

  if (options.advance === false) {
    state.lapAnalysis.activeDividerIndex = dividerIndex;
  } else {
    state.lapAnalysis.activeDividerIndex = nextUnpinnedDividerIndex(dividerIndex);
  }

  refreshLapAnalysis();
  setStatus(options.statusMessage ?? `Pinned divider ${dividerIndex + 1}.`, 'Lap analysis');
}

function clearDividerPin(dividerIndex) {
  state.lapAnalysis.dividerPins[dividerIndex] = null;
  state.lapAnalysis.activeDividerIndex = dividerIndex;
  refreshLapAnalysis();
  setStatus(`Cleared divider ${dividerIndex + 1}.`, 'Lap analysis');
}

function findNearestTrackPointToLngLat(lngLat) {
  if (!state.lapAnalysis.trackSamples.length) {
    return null;
  }

  let nearestSample = null;

  for (const sample of state.lapAnalysis.trackSamples) {
    const distanceMeters = haversineDistanceMeters(
      lngLat.lat,
      lngLat.lng,
      sample.latitude,
      sample.longitude,
    );

    if (!nearestSample || distanceMeters < nearestSample.distanceMeters) {
      nearestSample = {
        pointIndex: sample.pointIndex,
        distanceMeters,
      };
    }
  }

  return nearestSample;
}

function handleAnalysisTabClick(event) {
  const nextTab = event.currentTarget.dataset.tab;
  if (!nextTab || nextTab === state.activeTab) {
    return;
  }

  state.activeTab = nextTab;
  renderLapAnalysis();
  refreshLapCrossingsSource();
  updateDividerMarkers();
}

function handleLapDividerListClick(event) {
  const actionButton = event.target.closest('[data-action]');
  if (!actionButton) {
    return;
  }

  const dividerIndex = Number(actionButton.dataset.dividerIndex);
  if (!Number.isInteger(dividerIndex)) {
    return;
  }

  if (actionButton.dataset.action === 'arm-divider') {
    state.lapAnalysis.activeDividerIndex = state.lapAnalysis.activeDividerIndex === dividerIndex
      ? null
      : dividerIndex;
    renderLapAnalysis();
    updateDividerMarkers();
    return;
  }

  if (actionButton.dataset.action === 'clear-divider') {
    clearDividerPin(dividerIndex);
  }
}

function handleLapSectorCountChange(event) {
  const nextCount = Number(event.target.value);
  if (!Number.isInteger(nextCount) || nextCount < 1) {
    return;
  }

  resizeDividerPins(nextCount);
  refreshLapAnalysis();
  setStatus(`Lap analysis now expects ${nextCount} sector divider${nextCount === 1 ? '' : 's'}.`, 'Lap analysis');
}

function computeMetricRanges(points) {
  const ranges = {
    speedMaxMph: 0,
    accelMaxAbs: 0,
    accelColorMaxAbs: 0,
    qualityMax: 0,
    snrMax: 0,
    satelliteMax: 0,
    radar: {
      speedMaxMph: 0,
      longAccelMaxAbs: 0,
      latAccelMaxAbs: 0,
    },
  };
  const accelValues = [];

  for (const point of points) {
    const speedValue = speedMph(point);
    const accelValue = absoluteOrNull(point.accel_magnitude_mps2);
    const qualityValue = point.gps_lock_quality;
    const snrValue = point.average_snr_dbhz;
    const satelliteValue = satelliteCountValue(point);
    const longAccel = absoluteOrNull(point.longitudinal_accel_mps2);
    const latAccel = absoluteOrNull(point.latitudinal_accel_mps2);

    if (finiteNumber(speedValue)) {
      ranges.speedMaxMph = Math.max(ranges.speedMaxMph, speedValue);
      ranges.radar.speedMaxMph = Math.max(ranges.radar.speedMaxMph, speedValue);
    }
    if (finiteNumber(accelValue)) {
      ranges.accelMaxAbs = Math.max(ranges.accelMaxAbs, accelValue);
      accelValues.push(accelValue);
    }
    if (finiteNumber(qualityValue)) {
      ranges.qualityMax = Math.max(ranges.qualityMax, qualityValue);
    }
    if (finiteNumber(snrValue)) {
      ranges.snrMax = Math.max(ranges.snrMax, snrValue);
    }
    if (finiteNumber(satelliteValue)) {
      ranges.satelliteMax = Math.max(ranges.satelliteMax, satelliteValue);
    }
    if (finiteNumber(longAccel)) {
      ranges.radar.longAccelMaxAbs = Math.max(ranges.radar.longAccelMaxAbs, longAccel);
    }
    if (finiteNumber(latAccel)) {
      ranges.radar.latAccelMaxAbs = Math.max(ranges.radar.latAccelMaxAbs, latAccel);
    }
  }

  ranges.accelColorMaxAbs = accelValues.length
    ? percentileValue(accelValues, ACCEL_COLOR_PERCENTILE)
    : 0;

  if (ranges.accelColorMaxAbs <= 0) {
    ranges.accelColorMaxAbs = ranges.accelMaxAbs;
  }

  return ranges;
}

function handleMapLoad() {
  state.mapReady = true;
  initialize3DLayers();
  applyMapPresentation();
  renderLapAnalysis();
  refreshLapCrossingsSource();
  updateDividerMarkers();
  if (state.currentReport) {
    renderMap(state.currentReport);
  }
}

function initialize3DLayers() {
  if (!map.getSource(TRACK_SOURCE_ID)) {
    map.addSource(TRACK_SOURCE_ID, {
      type: 'geojson',
      data: featureCollection(),
    });
  }
  if (!map.getSource(BARS_SOURCE_ID)) {
    map.addSource(BARS_SOURCE_ID, {
      type: 'geojson',
      data: featureCollection(),
    });
  }
  if (!map.getSource(HITS_SOURCE_ID)) {
    map.addSource(HITS_SOURCE_ID, {
      type: 'geojson',
      data: featureCollection(),
    });
  }
  if (!map.getSource(MARKERS_SOURCE_ID)) {
    map.addSource(MARKERS_SOURCE_ID, {
      type: 'geojson',
      data: featureCollection(),
    });
  }
  if (!map.getSource(SELECTED_SOURCE_ID)) {
    map.addSource(SELECTED_SOURCE_ID, {
      type: 'geojson',
      data: featureCollection(),
    });
  }
  if (!map.getSource(LAP_CROSSINGS_SOURCE_ID)) {
    map.addSource(LAP_CROSSINGS_SOURCE_ID, {
      type: 'geojson',
      data: featureCollection(),
    });
  }

  if (!map.getLayer(TRACK_GLOW_LAYER_ID)) {
    map.addLayer({
      id: TRACK_GLOW_LAYER_ID,
      type: 'line',
      source: TRACK_SOURCE_ID,
      paint: {
        'line-color': '#fff7e3',
        'line-width': 10,
        'line-opacity': 0.16,
        'line-blur': 1.4,
      },
    });
  }

  if (!map.getLayer(TRACK_LINE_LAYER_ID)) {
    map.addLayer({
      id: TRACK_LINE_LAYER_ID,
      type: 'line',
      source: TRACK_SOURCE_ID,
      paint: {
        'line-color': '#f6efe3',
        'line-width': 3,
        'line-opacity': 0.92,
      },
    });
  }

  if (!map.getLayer(BARS_LAYER_ID)) {
    map.addLayer({
      id: BARS_LAYER_ID,
      type: 'fill-extrusion',
      source: BARS_SOURCE_ID,
      paint: {
        'fill-extrusion-color': ['get', 'color'],
        'fill-extrusion-height': ['get', 'height'],
        'fill-extrusion-base': 0,
        'fill-extrusion-opacity': 0.78,
        'fill-extrusion-vertical-gradient': true,
      },
    });
  }

  if (!map.getLayer(SPEED_HEATMAP_LAYER_ID)) {
    map.addLayer(
      {
        id: SPEED_HEATMAP_LAYER_ID,
        type: 'heatmap',
        source: HITS_SOURCE_ID,
        layout: {
          visibility: 'none',
        },
        paint: {
          'heatmap-weight': [
            'interpolate',
            ['linear'],
            ['coalesce', ['get', 'speedMph'], 0],
            0, 0,
            6, 0.18,
            19, 0.48,
            37, 0.82,
            62, 1,
          ],
          'heatmap-intensity': [
            'interpolate',
            ['linear'],
            ['zoom'],
            8, 0.5,
            12, 0.85,
            16, 1.2,
          ],
          'heatmap-radius': [
            'interpolate',
            ['linear'],
            ['zoom'],
            8, 8,
            12, 14,
            16, 24,
          ],
          'heatmap-color': [
            'interpolate',
            ['linear'],
            ['heatmap-density'],
            0, 'rgba(30,58,138,0)',
            0.15, '#1e3a8a',
            0.32, '#2563eb',
            0.5, '#0ea5a4',
            0.68, '#4ade80',
            0.84, '#facc15',
            0.94, '#fb923c',
            1, '#dc2626',
          ],
          'heatmap-opacity': 0.88,
        },
      },
      TRACK_GLOW_LAYER_ID,
    );
  }

  if (!map.getLayer(HITS_LAYER_ID)) {
    map.addLayer({
      id: HITS_LAYER_ID,
      type: 'circle',
      source: HITS_SOURCE_ID,
      paint: {
        'circle-radius': 8,
        'circle-color': '#ffffff',
        'circle-opacity': 0.01,
      },
    });
  }

  if (!map.getLayer(MARKERS_LAYER_ID)) {
    map.addLayer({
      id: MARKERS_LAYER_ID,
      type: 'circle',
      source: MARKERS_SOURCE_ID,
      paint: {
        'circle-radius': 6,
        'circle-color': ['get', 'color'],
        'circle-stroke-color': '#0f1b1a',
        'circle-stroke-width': 2,
      },
    });
  }

  if (!map.getLayer(SELECTED_LAYER_ID)) {
    map.addLayer({
      id: SELECTED_LAYER_ID,
      type: 'circle',
      source: SELECTED_SOURCE_ID,
      paint: {
        'circle-radius': 9,
        'circle-color': '#fff7e3',
        'circle-stroke-color': '#f16839',
        'circle-stroke-width': 3,
      },
    });
  }

  if (!map.getLayer(LAP_CROSSINGS_LAYER_ID)) {
    map.addLayer({
      id: LAP_CROSSINGS_LAYER_ID,
      type: 'circle',
      source: LAP_CROSSINGS_SOURCE_ID,
      layout: {
        visibility: 'none',
      },
      paint: {
        'circle-radius': 5,
        'circle-color': ['get', 'color'],
        'circle-stroke-color': '#fff7e3',
        'circle-stroke-width': 1.5,
        'circle-opacity': 0.96,
      },
    });
  }
}

function emptyMapSources() {
  setSourceData(TRACK_SOURCE_ID, featureCollection());
  setSourceData(BARS_SOURCE_ID, featureCollection());
  setSourceData(HITS_SOURCE_ID, featureCollection());
  setSourceData(MARKERS_SOURCE_ID, featureCollection());
  setSourceData(SELECTED_SOURCE_ID, featureCollection());
  setSourceData(LAP_CROSSINGS_SOURCE_ID, featureCollection());
}

function clearTrack() {
  state.selectedPointIndex = null;
  elements.pointDetailOverlay.hidden = true;
  removeDividerMarkers();
  emptyMapSources();
  hideSpeedLegend();
}

function buildTrackFeatureCollection(points) {
  if (!points || points.length < 2) {
    return featureCollection();
  }

  return featureCollection([
    {
      type: 'Feature',
      properties: {
        pointCount: points.length,
      },
      geometry: {
        type: 'LineString',
        coordinates: points.map((point) => [point.longitude, point.latitude]),
      },
    },
  ]);
}

function buildHitFeatureCollection(points) {
  const features = [];
  for (const [index, point] of points.entries()) {
    const pointSpeedMph = speedMph(point);
    features.push({
      type: 'Feature',
      properties: {
        pointIndex: index,
        speedMph: finiteNumber(pointSpeedMph) ? pointSpeedMph : 0,
      },
      geometry: {
        type: 'Point',
        coordinates: [point.longitude, point.latitude],
      },
    });
  }
  return featureCollection(features);
}

function buildMarkerFeatureCollection(points) {
  if (!points.length) {
    return featureCollection();
  }

  const features = [
    {
      type: 'Feature',
      properties: {
        pointIndex: 0,
        role: 'start',
        color: '#16a34a',
      },
      geometry: {
        type: 'Point',
        coordinates: [points[0].longitude, points[0].latitude],
      },
    },
  ];

  if (points.length > 1) {
    features.push({
      type: 'Feature',
      properties: {
        pointIndex: points.length - 1,
        role: 'end',
        color: '#f16839',
      },
      geometry: {
        type: 'Point',
        coordinates: [points[points.length - 1].longitude, points[points.length - 1].latitude],
      },
    });
  }

  return featureCollection(features);
}

function buildSelectedFeatureCollection() {
  if (
    !state.currentReport
    || !Number.isInteger(state.selectedPointIndex)
    || !state.currentReport.points[state.selectedPointIndex]
  ) {
    return featureCollection();
  }

  const point = state.currentReport.points[state.selectedPointIndex];
  return featureCollection([
    {
      type: 'Feature',
      properties: {
        pointIndex: state.selectedPointIndex,
      },
      geometry: {
        type: 'Point',
        coordinates: [point.longitude, point.latitude],
      },
    },
  ]);
}

function localOffsetPoint(point, eastMeters, northMeters) {
  const latitudeDegreesPerMeter = 1 / 111320;
  const longitudeDegreesPerMeter = 1 / (111320 * Math.max(Math.cos((point.latitude * Math.PI) / 180), 0.00001));
  return [
    point.longitude + (eastMeters * longitudeDegreesPerMeter),
    point.latitude + (northMeters * latitudeDegreesPerMeter),
  ];
}

function overlayOffsetMeters(overlayIndex, overlayCount) {
  if (overlayCount <= 1) {
    return { east: 0, north: 0 };
  }
  const radius = overlayCount === 2 ? 6 : 8;
  const angle = (-Math.PI / 2) + ((Math.PI * 2 * overlayIndex) / overlayCount);
  return {
    east: Math.cos(angle) * radius,
    north: Math.sin(angle) * radius,
  };
}

function squarePolygon(point, eastMeters, northMeters, halfSizeMeters) {
  const corners = [
    localOffsetPoint(point, eastMeters - halfSizeMeters, northMeters - halfSizeMeters),
    localOffsetPoint(point, eastMeters + halfSizeMeters, northMeters - halfSizeMeters),
    localOffsetPoint(point, eastMeters + halfSizeMeters, northMeters + halfSizeMeters),
    localOffsetPoint(point, eastMeters - halfSizeMeters, northMeters + halfSizeMeters),
    localOffsetPoint(point, eastMeters - halfSizeMeters, northMeters - halfSizeMeters),
  ];

  return {
    type: 'Polygon',
    coordinates: [corners],
  };
}

function barHeightMeters(value, maximum, maxHeightMeters) {
  const normalized = normalizeAgainst(value, maximum);
  if (!finiteNumber(normalized)) {
    return null;
  }
  return 6 + (normalized * maxHeightMeters);
}

function buildBarsFeatureCollection(points) {
  const overlays = activeOverlayIds();
  if (!points.length || !overlays.length || !state.metricRanges) {
    return featureCollection();
  }

  const features = [];
  const sampleStep = Math.max(1, Math.ceil(points.length / 1200));
  const lastIndex = points.length - 1;
  const barSizeScale = state.barSizeScale;

  for (let pointIndex = 0; pointIndex <= lastIndex; pointIndex += sampleStep) {
    const point = points[pointIndex];

    overlays.forEach((overlayId, overlayIndex) => {
      const overlay = overlayDefinitions[overlayId];
      const heightValue = overlay.heightValue(point);
      const heightMaximum = state.metricRanges[overlay.heightRangeKey];
      const colorValue = overlay.colorValue(point);
      const colorMaximum = state.metricRanges[overlay.colorRangeKey];
      const colorNormalized = normalizeAgainst(colorValue, colorMaximum);
      const height = barHeightMeters(heightValue, heightMaximum, overlay.maxHeightMeters);
      if (!finiteNumber(height)) {
        return;
      }

      const offset = overlayOffsetMeters(overlayIndex, overlays.length);
      const color = paletteColor(colorNormalized, overlay.palette);
      const halfSizeMeters = (overlays.length > 1 ? 3.4 : 4.5) * barSizeScale;

      features.push({
        type: 'Feature',
        properties: {
          pointIndex,
          overlayId,
          label: overlay.label,
          color,
          colorMetricValue: finiteNumber(colorValue) ? colorValue : null,
          colorMetricNormalized: finiteNumber(colorNormalized) ? colorNormalized : null,
          colorMetricRange: finiteNumber(colorMaximum) ? colorMaximum : null,
          height,
        },
        geometry: squarePolygon(
          point,
          offset.east * barSizeScale,
          offset.north * barSizeScale,
          halfSizeMeters,
        ),
      });
    });
  }

  if (lastIndex % sampleStep !== 0 && points[lastIndex]) {
    const lastPoint = points[lastIndex];
    overlays.forEach((overlayId, overlayIndex) => {
      const overlay = overlayDefinitions[overlayId];
      const heightValue = overlay.heightValue(lastPoint);
      const heightMaximum = state.metricRanges[overlay.heightRangeKey];
      const colorValue = overlay.colorValue(lastPoint);
      const colorMaximum = state.metricRanges[overlay.colorRangeKey];
      const colorNormalized = normalizeAgainst(colorValue, colorMaximum);
      const height = barHeightMeters(heightValue, heightMaximum, overlay.maxHeightMeters);
      if (!finiteNumber(height)) {
        return;
      }

      const offset = overlayOffsetMeters(overlayIndex, overlays.length);
      const color = paletteColor(colorNormalized, overlay.palette);
      const halfSizeMeters = (overlays.length > 1 ? 3.4 : 4.5) * barSizeScale;

      features.push({
        type: 'Feature',
        properties: {
          pointIndex: lastIndex,
          overlayId,
          label: overlay.label,
          color,
          colorMetricValue: finiteNumber(colorValue) ? colorValue : null,
          colorMetricNormalized: finiteNumber(colorNormalized) ? colorNormalized : null,
          colorMetricRange: finiteNumber(colorMaximum) ? colorMaximum : null,
          height,
        },
        geometry: squarePolygon(
          lastPoint,
          offset.east * barSizeScale,
          offset.north * barSizeScale,
          halfSizeMeters,
        ),
      });
    });
  }

  return featureCollection(features);
}

function refreshSelectedPointSource() {
  setSourceData(SELECTED_SOURCE_ID, buildSelectedFeatureCollection());
}

function refreshBarSource() {
  if (!state.currentReport) {
    setSourceData(BARS_SOURCE_ID, featureCollection());
    elements.barSizeRange.disabled = true;
    return;
  }

  elements.barSizeRange.disabled = activeOverlayIds().length === 0;
  const barsData = buildBarsFeatureCollection(state.currentReport.points);
  setSourceData(BARS_SOURCE_ID, barsData);

  if (state.mapReady && map.getLayer(BARS_LAYER_ID)) {
    map.setLayoutProperty(BARS_LAYER_ID, 'visibility', barsData.features.length ? 'visible' : 'none');
  }

  if (usingVelocityFallback()) {
    showSpeedLegend(state.metricRanges, 'Velocity heatmap');
  } else if (state.activeVisualizations.has('speedAccel')) {
    showSpeedLegend(state.metricRanges, 'Speed / accel');
  } else {
    hideSpeedLegend();
  }
}

function refreshMapLayers(report) {
  if (!state.mapReady) {
    return;
  }

  applyMapPresentation();
  setSourceData(TRACK_SOURCE_ID, buildTrackFeatureCollection(report.points));
  setSourceData(HITS_SOURCE_ID, buildHitFeatureCollection(report.points));
  setSourceData(MARKERS_SOURCE_ID, buildMarkerFeatureCollection(report.points));
  refreshBarSource();
  refreshSelectedPointSource();
}

function sentenceLabel(sentenceType) {
  return sentenceType.startsWith('GP') || sentenceType.startsWith('GN') || sentenceType.startsWith('GL')
    ? sentenceType.slice(2)
    : sentenceType;
}

function renderSentenceCounts(sentenceTypeCounts) {
  const entries = Object.entries(sentenceTypeCounts || {});
  if (!entries.length) {
    elements.sentenceCounts.innerHTML = '<div class="sentence-item"><strong>Waiting</strong><span>Open a file to see the sentence mix.</span></div>';
    elements.rmcShare.textContent = 'Waiting';
    return;
  }

  const total = entries.reduce((sum, [, count]) => sum + count, 0);
  const rmcTotal = entries
    .filter(([type]) => sentenceLabel(type) === 'RMC')
    .reduce((sum, [, count]) => sum + count, 0);

  elements.rmcShare.textContent = `${formatNumber((rmcTotal / total) * 100, 0)}% RMC`;
  elements.sentenceCounts.innerHTML = entries
    .map(([type, count]) => `
      <div class="sentence-item">
        <strong>${escapeHtml(type)}</strong>
        <span>${formatNumber(count)} sentences</span>
      </div>
    `)
    .join('');
}

function renderSummary(report) {
  const points = report.points || [];
  const startPoint = report.firstFix;
  const endPoint = report.lastFix;
  const topSpeedValue = points.reduce((maximum, point) => {
    const current = speedMph(point);
    return finiteNumber(current) ? Math.max(maximum, current) : maximum;
  }, 0);

  elements.fixCount.textContent = formatNumber(report.fixCount || 0);
  elements.distanceKm.textContent = `${formatNumber(report.distanceKilometers || 0, 3)} km`;
  elements.sentenceCount.textContent = formatNumber(report.sentenceCount || 0);
  elements.checksumCount.textContent = formatNumber(report.invalidChecksumCount || 0);
  elements.timeSpan.textContent = formatDuration(startPoint?.timestamp, endPoint?.timestamp);
  elements.startFix.textContent = startPoint ? formatTimestamp(startPoint.timestamp) : '--';
  elements.endFix.textContent = endPoint ? formatTimestamp(endPoint.timestamp) : '--';
  elements.trackBounds.textContent = report.bounds
    ? `${formatNumber(report.bounds.minLatitude, 5)}, ${formatNumber(report.bounds.minLongitude, 5)} -> ${formatNumber(report.bounds.maxLatitude, 5)}, ${formatNumber(report.bounds.maxLongitude, 5)}`
    : '--';
  elements.topSpeed.textContent = topSpeedValue > 0 ? `${formatNumber(topSpeedValue, 1)} mph` : '--';
  renderSentenceCounts(report.sentenceTypeCounts);
}

function pointSummaryCards(point) {
  const summary = [
    {
      label: 'Coordinates',
      value: formatCoordinatePair(point),
    },
    {
      label: 'GPS lock quality',
      value: lockQualityText(point),
    },
    {
      label: 'Satellite count',
      value: finiteNumber(point.satellite_count_used)
        ? `${formatNumber(point.satellite_count_used)} used / ${finiteNumber(point.satellite_count_in_view) ? formatNumber(point.satellite_count_in_view) : '--'} in view`
        : (finiteNumber(point.satellite_count_in_view) ? `${formatNumber(point.satellite_count_in_view)} in view` : '--'),
    },
    {
      label: 'Average SNR',
      value: finiteNumber(point.average_snr_dbhz) ? `${formatNumber(point.average_snr_dbhz, 1)} dB-Hz` : '--',
    },
    {
      label: 'Altitude',
      value: finiteNumber(point.altitude_m) ? `${formatNumber(point.altitude_m, 1)} m` : '--',
    },
    {
      label: 'HDOP',
      value: finiteNumber(point.hdop) ? formatNumber(point.hdop, 2) : '--',
    },
  ];

  return summary
    .map(
      (item) => `
        <div class="point-summary-card">
          <strong>${escapeHtml(item.label)}</strong>
          <span>${escapeHtml(item.value)}</span>
        </div>
      `,
    )
    .join('');
}

function radarMetrics(point) {
  const radarExtents = state.metricRanges?.radar || {};
  const speedValue = speedMph(point);
  const longAccel = point.longitudinal_accel_mps2;
  const latAccel = point.latitudinal_accel_mps2;
  const qualityValue = point.gps_lock_quality;
  const snrValue = point.average_snr_dbhz;
  const satelliteValue = satelliteCountValue(point);
  const hdopValue = point.hdop;
  const accelMagnitude = absoluteOrNull(point.accel_magnitude_mps2);

  return [
    {
      axisLabel: 'Speed',
      label: 'Speed',
      displayValue: finiteNumber(speedValue) ? `${formatNumber(speedValue, 1)} mph` : '--',
      normalized: normalizeAgainst(speedValue, radarExtents.speedMaxMph) ?? 0,
    },
    {
      axisLabel: 'Long acc',
      label: 'Longitudinal accel',
      displayValue: finiteNumber(longAccel) ? `${formatNumber(longAccel, 2)} m/s^2` : '--',
      normalized: normalizeAgainst(absoluteOrNull(longAccel), radarExtents.longAccelMaxAbs) ?? 0,
    },
    {
      axisLabel: 'Lat acc',
      label: 'Latitudinal accel',
      displayValue: finiteNumber(latAccel) ? `${formatNumber(latAccel, 2)} m/s^2` : '--',
      normalized: normalizeAgainst(absoluteOrNull(latAccel), radarExtents.latAccelMaxAbs) ?? 0,
    },
    {
      axisLabel: 'Quality',
      label: 'GPS lock quality',
      displayValue: lockQualityText(point),
      normalized: normalizeAgainst(qualityValue, state.metricRanges?.qualityMax) ?? 0,
    },
    {
      axisLabel: 'SNR',
      label: 'Average SNR',
      displayValue: finiteNumber(snrValue) ? `${formatNumber(snrValue, 1)} dB-Hz` : '--',
      normalized: normalizeAgainst(snrValue, state.metricRanges?.snrMax) ?? 0,
    },
    {
      axisLabel: 'Sats',
      label: 'Satellite count',
      displayValue: finiteNumber(satelliteValue) ? formatNumber(satelliteValue) : '--',
      normalized: normalizeAgainst(satelliteValue, state.metricRanges?.satelliteMax) ?? 0,
    },
    {
      axisLabel: 'Accel',
      label: 'Accel magnitude',
      displayValue: finiteNumber(accelMagnitude) ? `${formatNumber(accelMagnitude, 2)} m/s^2` : '--',
      normalized: normalizeAgainst(accelMagnitude, state.metricRanges?.accelMaxAbs) ?? 0,
    },
    {
      axisLabel: 'HDOP',
      label: 'HDOP',
      displayValue: finiteNumber(hdopValue) ? formatNumber(hdopValue, 2) : '--',
      normalized: finiteNumber(hdopValue) ? 1 - (normalizeAgainst(hdopValue, Math.max(state.metricRanges?.qualityMax || 1, 4)) ?? 0) : 0,
    },
  ];
}

function polarPoint(centerX, centerY, radius, angle) {
  return {
    x: centerX + (Math.cos(angle) * radius),
    y: centerY + (Math.sin(angle) * radius),
  };
}

function polygonPath(points) {
  return points.map((point) => `${point.x.toFixed(2)},${point.y.toFixed(2)}`).join(' ');
}

function radarSvg(metrics) {
  const size = 336;
  const center = size / 2;
  const radius = 108;
  const axisCount = metrics.length;
  const rings = 4;

  const gridPolygons = Array.from({ length: rings }, (_value, ringIndex) => {
    const ringRadius = radius * ((ringIndex + 1) / rings);
    const ringPoints = metrics.map((_metric, metricIndex) => {
      const angle = (-Math.PI / 2) + ((Math.PI * 2 * metricIndex) / axisCount);
      return polarPoint(center, center, ringRadius, angle);
    });
    return `<polygon class="radar-grid" points="${polygonPath(ringPoints)}"></polygon>`;
  }).join('');

  const axisLines = metrics.map((metric, metricIndex) => {
    const angle = (-Math.PI / 2) + ((Math.PI * 2 * metricIndex) / axisCount);
    const edgePoint = polarPoint(center, center, radius, angle);
    const labelPoint = polarPoint(center, center, radius + 30, angle);
    return `
      <line class="radar-axis" x1="${center}" y1="${center}" x2="${edgePoint.x}" y2="${edgePoint.y}"></line>
      <text class="radar-label" x="${labelPoint.x}" y="${labelPoint.y}">${escapeHtml(metric.axisLabel)}</text>
    `;
  }).join('');

  const dataPoints = metrics.map((metric, metricIndex) => {
    const angle = (-Math.PI / 2) + ((Math.PI * 2 * metricIndex) / axisCount);
    return polarPoint(center, center, radius * metric.normalized, angle);
  });

  const dataCircles = dataPoints
    .map((point) => `<circle class="radar-point" cx="${point.x}" cy="${point.y}" r="3.5"></circle>`)
    .join('');

  return `
    <svg viewBox="0 0 ${size} ${size}" role="img" aria-label="Octagon point quality chart">
      ${gridPolygons}
      ${axisLines}
      <polygon class="radar-shape" points="${polygonPath(dataPoints)}"></polygon>
      ${dataCircles}
    </svg>
  `;
}

function radarValueCards(metrics) {
  return metrics
    .map(
      (metric) => `
        <div class="point-value-card">
          <strong>${escapeHtml(metric.label)}</strong>
          <span>${escapeHtml(metric.displayValue)}</span>
        </div>
      `,
    )
    .join('');
}

function openPointDetail(point, index) {
  state.selectedPointIndex = index;
  const metrics = radarMetrics(point);

  elements.pointDetailTitle.textContent = `Fix ${index + 1}`;
  elements.pointDetailTime.textContent = formatTimestamp(point.timestamp);
  elements.pointDetailSubtitle.textContent = `${formatCoordinatePair(point)} | Active overlays: ${overlaySummaryText()}`;
  elements.pointDetailSummary.innerHTML = pointSummaryCards(point);
  elements.pointRadarChart.innerHTML = radarSvg(metrics);
  elements.pointRadarValues.innerHTML = radarValueCards(metrics);
  elements.pointDetailOverlay.hidden = false;

  refreshSelectedPointSource();
}

function closePointDetail() {
  state.selectedPointIndex = null;
  elements.pointDetailOverlay.hidden = true;
  refreshSelectedPointSource();
}

function findNearestPointIndex(screenPoint, maxPixelDistance = 18) {
  if (!state.currentReport || !state.currentReport.points.length || !state.mapReady) {
    return null;
  }

  let nearestIndex = null;
  let nearestDistanceSquared = maxPixelDistance * maxPixelDistance;

  for (const [index, point] of state.currentReport.points.entries()) {
    const projected = map.project([point.longitude, point.latitude]);
    const distanceSquared = ((projected.x - screenPoint.x) ** 2) + ((projected.y - screenPoint.y) ** 2);
    if (distanceSquared < nearestDistanceSquared) {
      nearestDistanceSquared = distanceSquared;
      nearestIndex = index;
    }
  }

  return nearestIndex;
}

function handleMapClick(event) {
  if (!state.currentReport || !state.currentReport.points.length || !state.mapReady) {
    return;
  }

  if (state.activeTab === 'lapAnalysis' && Number.isInteger(state.lapAnalysis.activeDividerIndex)) {
    const nearestTrackPoint = findNearestTrackPointToLngLat(event.lngLat);
    if (!nearestTrackPoint || nearestTrackPoint.distanceMeters > DIVIDER_SNAP_MAX_DISTANCE_METERS) {
      setStatus(
        `No nearby track point found for divider ${state.lapAnalysis.activeDividerIndex + 1}. Click closer to the imported line.`,
        'Lap analysis',
      );
      return;
    }

    const snappedPoint = state.currentReport.points[nearestTrackPoint.pointIndex];
    setDividerPin(
      state.lapAnalysis.activeDividerIndex,
      {
        latitude: snappedPoint.latitude,
        longitude: snappedPoint.longitude,
        pointIndex: nearestTrackPoint.pointIndex,
      },
      {
        statusMessage: `Pinned divider ${state.lapAnalysis.activeDividerIndex + 1} using imported fix ${nearestTrackPoint.pointIndex + 1}.`,
      },
    );
    return;
  }

  const features = map.queryRenderedFeatures(event.point, {
    layers: [BARS_LAYER_ID, HITS_LAYER_ID, MARKERS_LAYER_ID, SELECTED_LAYER_ID],
  });

  for (const feature of features) {
    const pointIndex = Number(feature.properties?.pointIndex);
    if (Number.isInteger(pointIndex) && state.currentReport.points[pointIndex]) {
      openPointDetail(state.currentReport.points[pointIndex], pointIndex);
      return;
    }
  }

  const nearestIndex = findNearestPointIndex(event.point);
  if (nearestIndex !== null) {
    openPointDetail(state.currentReport.points[nearestIndex], nearestIndex);
  }
}

function renderMap(report) {
  clearTrack();
  const pose = cameraPoseForCurrentMode();

  if (!report.points.length) {
    if (state.mapReady) {
      applyMapPresentation();
      map.easeTo({
        center: [defaultCenter[1], defaultCenter[0]],
        zoom: defaultZoom,
        pitch: pose.pitch,
        bearing: pose.bearing,
        duration: 0,
      });
    }
    elements.mapOverlayMessage.textContent = 'The log parsed, but there were no valid position fixes to plot.';
    elements.mapOverlayMessage.hidden = false;
    elements.mapTitle.textContent = shortFileName(report.inputPath);
    setStatus('No valid fixes were found in the selected log.', 'No fixes');
    return;
  }

  elements.mapOverlayMessage.hidden = true;
  state.metricRanges = computeMetricRanges(report.points);
  refreshMapLayers(report);

  if (state.mapReady && report.bounds) {
    const minLongitude = report.bounds.minLongitude;
    const minLatitude = report.bounds.minLatitude;
    const maxLongitude = report.bounds.maxLongitude;
    const maxLatitude = report.bounds.maxLatitude;

    if (
      finiteNumber(minLongitude)
      && finiteNumber(minLatitude)
      && finiteNumber(maxLongitude)
      && finiteNumber(maxLatitude)
      && (minLongitude !== maxLongitude || minLatitude !== maxLatitude)
    ) {
      const bounds = new maplibregl.LngLatBounds(
        [minLongitude, minLatitude],
        [maxLongitude, maxLatitude],
      );
      map.fitBounds(bounds, {
        padding: 60,
        pitch: pose.pitch,
        bearing: pose.bearing,
        duration: 0,
        maxZoom: 16,
      });
    } else if (report.points[0]) {
      map.easeTo({
        center: [report.points[0].longitude, report.points[0].latitude],
        zoom: 16,
        pitch: pose.pitch,
        bearing: pose.bearing,
        duration: 0,
      });
    }
  }

  elements.mapTitle.textContent = shortFileName(report.inputPath);
  setStatus(
    `Parsed ${formatNumber(report.fixCount)} fixes from ${shortFileName(report.inputPath)}. Active overlays: ${overlaySummaryText()}.`,
    'Track ready',
  );
}

const RTK_QUALITY_LABELS = { 0: 'No fix', 1: 'Float', 2: 'Fixed' };

function renderPqtmData(pqtmData) {
  if (!pqtmData) {
    elements.pqtmRtkChip.textContent = 'No data';
    elements.pqtmRtkChip.className = 'chip muted';
    elements.pqtmVehattCount.textContent = '-';
    elements.pqtmTarCount.textContent = '-';
    elements.pqtmTarFixed.textContent = '-';
    elements.pqtmTarFloat.textContent = '-';
    elements.pqtmHeading.textContent = '-';
    elements.pqtmRollPitch.textContent = '-';
    elements.pqtmDist.textContent = '-';
    return;
  }

  const fixedCount = pqtmData.tar_fixed_count || 0;
  const floatCount = pqtmData.tar_float_count || 0;
  if (fixedCount > 0) {
    elements.pqtmRtkChip.textContent = 'RTK fixed';
    elements.pqtmRtkChip.className = 'chip';
  } else if (floatCount > 0) {
    elements.pqtmRtkChip.textContent = 'RTK float';
    elements.pqtmRtkChip.className = 'chip muted';
  } else {
    elements.pqtmRtkChip.textContent = 'No RTK lock';
    elements.pqtmRtkChip.className = 'chip muted';
  }

  elements.pqtmVehattCount.textContent = formatNumber(pqtmData.vehatt_count || 0);
  elements.pqtmTarCount.textContent = formatNumber(pqtmData.tar_count || 0);
  elements.pqtmTarFixed.textContent = formatNumber(fixedCount);
  elements.pqtmTarFloat.textContent = formatNumber(floatCount);

  const vehatt = pqtmData.latest_vehatt;
  if (vehatt && vehatt.heading_deg != null) {
    elements.pqtmHeading.textContent = `${formatNumber(vehatt.heading_deg, 2)}°`;
  } else {
    elements.pqtmHeading.textContent = '--';
  }

  if (vehatt && (vehatt.roll_deg != null || vehatt.pitch_deg != null)) {
    const roll = vehatt.roll_deg != null ? `${formatNumber(vehatt.roll_deg, 2)}°` : '--';
    const pitch = vehatt.pitch_deg != null ? `${formatNumber(vehatt.pitch_deg, 2)}°` : '--';
    elements.pqtmRollPitch.textContent = `${roll} / ${pitch}`;
  } else {
    elements.pqtmRollPitch.textContent = '--';
  }

  const tar = pqtmData.latest_tar;
  if (tar && tar.horiz_dist_m != null) {
    elements.pqtmDist.textContent = `${formatNumber(tar.horiz_dist_m, 3)} m`;
  } else {
    elements.pqtmDist.textContent = '--';
  }
}

function renderRtcmData(rtcmData) {
  if (!rtcmData) {
    elements.rtcmChip.textContent = 'No data';
    elements.rtcmChip.className = 'chip muted';
    elements.rtcmFrameCount.textContent = '-';
    elements.rtcmCrcErrors.textContent = '-';
    elements.rtcmBytes.textContent = '-';
    elements.rtcmMsgTypes.innerHTML = '<div class="sentence-item"><strong>None</strong><span>No R&gt; rover lines found.</span></div>';
    return;
  }

  const frameCount = rtcmData.frame_count || 0;
  elements.rtcmChip.textContent = `${formatNumber(frameCount)} frames`;
  elements.rtcmChip.className = frameCount > 0 ? 'chip' : 'chip muted';
  elements.rtcmFrameCount.textContent = formatNumber(frameCount);
  elements.rtcmCrcErrors.textContent = formatNumber(rtcmData.crc_error_count || 0);
  elements.rtcmBytes.textContent = `${formatNumber(rtcmData.total_bytes || 0)} B`;

  const typeCounts = rtcmData.message_type_counts || {};
  const entries = Object.entries(typeCounts);
  if (!entries.length) {
    elements.rtcmMsgTypes.innerHTML = '<div class="sentence-item"><strong>No valid frames</strong><span>Sync bytes found but no valid RTCM3 frames decoded.</span></div>';
    return;
  }

  elements.rtcmMsgTypes.innerHTML = entries
    .sort(([, a], [, b]) => b.count - a.count)
    .map(([msgType, info]) => `
      <div class="sentence-item">
        <strong>${escapeHtml(String(msgType))} — ${escapeHtml(info.name)}</strong>
        <span>${formatNumber(info.count)} frames</span>
      </div>
    `)
    .join('');
}

function renderReport(report) {
  state.currentReport = report;
  state.currentFilePath = report.inputPath;
  elements.selectedFile.textContent = report.inputPath;
  resetLapAnalysisForReport(report);
  renderSummary(report);
  renderPqtmData(report.pqtmData || null);
  renderRtcmData(report.rtcmData || null);
  renderMap(report);
  setBusy(false);
}

async function openLogFile() {
  const filePath = await window.gpsMapApp.pickLogFile();
  if (!filePath) {
    return;
  }

  state.currentFilePath = filePath;
  elements.selectedFile.textContent = filePath;
  setBusy(true);
  setStatus(`Parsing ${shortFileName(filePath)}...`, 'Parsing');
  elements.mapOverlayMessage.hidden = false;
  elements.mapOverlayMessage.textContent = 'Parsing the capture and recovering valid fixes...';

  try {
    const report = await window.gpsMapApp.parseLogFile(filePath);
    renderReport(report);
  } catch (error) {
    state.currentReport = null;
    state.lapAnalysis.trackSamples = [];
    state.lapAnalysis.analysis = null;
    state.lapAnalysis.dividerPins = Array.from({ length: state.lapAnalysis.sectorCount }, () => null);
    state.lapAnalysis.activeDividerIndex = state.lapAnalysis.sectorCount ? 0 : null;
    clearTrack();
    renderLapAnalysis();
    elements.mapOverlayMessage.hidden = false;
    elements.mapOverlayMessage.textContent = 'The parser failed for this file.';
    setStatus(error.message, 'Error');
    setBusy(false);
  }
}

async function exportTrack(format) {
  if (!state.currentFilePath) {
    return;
  }

  setBusy(true);
  setStatus(`Exporting ${format.toUpperCase()}...`, 'Exporting');

  try {
    const outputPath = await window.gpsMapApp.exportTrack(state.currentFilePath, format);
    if (outputPath) {
      setStatus(`Exported ${format.toUpperCase()} to ${outputPath}`, 'Exported');
    } else if (state.currentReport) {
      setStatus('Export canceled.', 'Track ready');
    }
  } catch (error) {
    setStatus(error.message, 'Error');
  } finally {
    setBusy(false);
  }
}

function handleVisualizationChange(event) {
  const { checked, value } = event.target;
  if (checked) {
    state.activeVisualizations.add(value);
  } else {
    state.activeVisualizations.delete(value);
  }

  if (state.currentReport) {
    applyMapPresentation();
    const pose = cameraPoseForCurrentMode();
    map.easeTo({
      pitch: pose.pitch,
      bearing: pose.bearing,
      duration: 450,
      essential: true,
    });
    setStatus(
      `Parsed ${formatNumber(state.currentReport.fixCount)} fixes from ${shortFileName(state.currentReport.inputPath)}. Active overlays: ${overlaySummaryText()}.`,
      'Track ready',
    );
    refreshBarSource();
  }
}

function handleBarSizeChange(event) {
  state.barSizeScale = Number(event.target.value);
  renderBarSizeLabel();

  if (state.currentReport) {
    refreshBarSource();
  }
}

elements.openLogButton.addEventListener('click', openLogFile);
elements.exportGeoJsonButton.addEventListener('click', () => exportTrack('geojson'));
elements.exportKmlButton.addEventListener('click', () => exportTrack('kml'));
elements.barSizeRange.addEventListener('input', handleBarSizeChange);
elements.lapSectorCount.addEventListener('change', handleLapSectorCountChange);
elements.lapDividerList.addEventListener('click', handleLapDividerListClick);
elements.pointDetailClose.addEventListener('click', closePointDetail);
for (const button of elements.analysisTabButtons) {
  button.addEventListener('click', handleAnalysisTabClick);
}
for (const input of elements.visualizationInputs) {
  input.addEventListener('change', handleVisualizationChange);
}

setBusy(false);
renderBarSizeLabel();
hideSpeedLegend();
renderLapAnalysis();
renderSentenceCounts({});
