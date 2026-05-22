const defaultCenter = [34.0684, -118.4435];
const defaultZoom = 13;
const KPH_TO_MPH = 0.621371;
const KNOTS_TO_MPH = 1.150779;
const MPS_TO_MPH = 2.236936;
const ACCEL_COLOR_PERCENTILE = 0.95;
const speedHeatColors = ['#1e3a8a', '#2563eb', '#0ea5a4', '#4ade80', '#facc15', '#fb923c', '#dc2626'];
const qualityColors = ['#7f1d1d', '#b91c1c', '#ea580c', '#eab308', '#84cc16', '#16a34a'];
const visualizationOrder = ['speedAccel', 'lockQuality', 'snr', 'satelliteCount'];

const TRACK_SOURCE_ID = 'track-source';
const BARS_SOURCE_ID = 'bars-source';
const HITS_SOURCE_ID = 'hits-source';
const MARKERS_SOURCE_ID = 'markers-source';
const SELECTED_SOURCE_ID = 'selected-source';
const TERRAIN_SOURCE_ID = 'terrain-source';

const TRACK_GLOW_LAYER_ID = 'track-glow-layer';
const TRACK_LINE_LAYER_ID = 'track-line-layer';
const BARS_LAYER_ID = 'bars-layer';
const SPEED_HEATMAP_LAYER_ID = 'speed-heatmap-layer';
const HITS_LAYER_ID = 'hits-layer';
const MARKERS_LAYER_ID = 'markers-layer';
const SELECTED_LAYER_ID = 'selected-layer';

const MAP_3D_PITCH = 62;
const MAP_3D_BEARING = -24;
const MAP_2D_PITCH = 0;
const MAP_2D_BEARING = 0;

const elements = {
  openLogButton: document.getElementById('open-log-button'),
  selectedFile: document.getElementById('selected-file'),
  exportGeoJsonButton: document.getElementById('export-geojson-button'),
  exportKmlButton: document.getElementById('export-kml-button'),
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
  visualizationInputs: Array.from(document.querySelectorAll('.visualization-option input')),
  pointDetailOverlay: document.getElementById('point-detail-overlay'),
  pointDetailClose: document.getElementById('point-detail-close'),
  pointDetailTitle: document.getElementById('point-detail-title'),
  pointDetailTime: document.getElementById('point-detail-time'),
  pointDetailSubtitle: document.getElementById('point-detail-subtitle'),
  pointDetailSummary: document.getElementById('point-detail-summary'),
  pointRadarChart: document.getElementById('point-radar-chart'),
  pointRadarValues: document.getElementById('point-radar-values'),
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
  mapReady: false,
  currentFilePath: null,
  currentReport: null,
  selectedPointIndex: null,
  barSizeScale: 1,
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
}

function emptyMapSources() {
  setSourceData(TRACK_SOURCE_ID, featureCollection());
  setSourceData(BARS_SOURCE_ID, featureCollection());
  setSourceData(HITS_SOURCE_ID, featureCollection());
  setSourceData(MARKERS_SOURCE_ID, featureCollection());
  setSourceData(SELECTED_SOURCE_ID, featureCollection());
}

function clearTrack() {
  state.selectedPointIndex = null;
  elements.pointDetailOverlay.hidden = true;
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

function renderReport(report) {
  state.currentReport = report;
  state.currentFilePath = report.inputPath;
  elements.selectedFile.textContent = report.inputPath;
  renderSummary(report);
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
    clearTrack();
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
elements.pointDetailClose.addEventListener('click', closePointDetail);
for (const input of elements.visualizationInputs) {
  input.addEventListener('change', handleVisualizationChange);
}

setBusy(false);
renderBarSizeLabel();
hideSpeedLegend();
renderSentenceCounts({});
