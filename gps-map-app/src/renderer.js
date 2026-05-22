const defaultCenter = [34.0684, -118.4435];
const defaultZoom = 13;
const speedHeatColors = ['#1e3a8a', '#2563eb', '#0ea5a4', '#4ade80', '#facc15', '#fb923c', '#dc2626'];

const elements = {
  openLogButton: document.getElementById('open-log-button'),
  exportGeoJsonButton: document.getElementById('export-geojson-button'),
  exportKmlButton: document.getElementById('export-kml-button'),
  selectedFile: document.getElementById('selected-file'),
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
};

const state = {
  currentFilePath: null,
  currentReport: null,
  trackLayer: null,
};

const map = L.map('map', {
  preferCanvas: true,
  zoomControl: false,
}).setView(defaultCenter, defaultZoom);

L.control.zoom({ position: 'bottomright' }).addTo(map);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
  attribution: '&copy; OpenStreetMap contributors',
}).addTo(map);

function shortFileName(filePath) {
  return filePath.split(/[/\\]/).pop();
}

function formatNumber(value, digits = 0) {
  return new Intl.NumberFormat(undefined, {
    maximumFractionDigits: digits,
    minimumFractionDigits: digits,
  }).format(value);
}

function formatCoordinatePair(point) {
  return `${point.latitude.toFixed(6)}, ${point.longitude.toFixed(6)}`;
}

function formatTimestamp(timestamp) {
  if (!timestamp) {
    return 'Unknown';
  }

  return new Intl.DateTimeFormat(undefined, {
    dateStyle: 'medium',
    timeStyle: 'medium',
    timeZone: 'UTC',
  }).format(new Date(timestamp));
}

function formatDuration(startTimestamp, endTimestamp) {
  if (!startTimestamp || !endTimestamp) {
    return 'No fixes yet';
  }

  const start = new Date(startTimestamp);
  const end = new Date(endTimestamp);
  const durationSeconds = Math.max(0, Math.round((end - start) / 1000));
  const minutes = Math.floor(durationSeconds / 60);
  const seconds = durationSeconds % 60;

  if (minutes === 0) {
    return `${seconds}s span`;
  }

  return `${minutes}m ${seconds}s span`;
}

function topSpeedKph(points) {
  if (!points.length) {
    return null;
  }

  let highest = null;
  for (const point of points) {
    if (typeof point.speed_mps !== 'number') {
      continue;
    }

    const kph = point.speed_mps * 3.6;
    if (highest === null || kph > highest) {
      highest = kph;
    }
  }

  return highest;
}

function speedKph(point) {
  if (typeof point.speed_mps !== 'number' || !Number.isFinite(point.speed_mps)) {
    return null;
  }

  return point.speed_mps * 3.6;
}

function speedScale(points) {
  const speeds = points
    .map(speedKph)
    .filter((value) => value !== null);

  if (!speeds.length) {
    return null;
  }

  const min = Math.min(...speeds);
  const max = Math.max(...speeds);
  return {
    min,
    mid: min + ((max - min) / 2),
    max,
  };
}

function segmentSpeedKph(startPoint, endPoint) {
  const speeds = [speedKph(startPoint), speedKph(endPoint)].filter((value) => value !== null);
  if (!speeds.length) {
    return null;
  }

  return speeds.reduce((total, value) => total + value, 0) / speeds.length;
}

function speedColorIndex(speed, scale) {
  if (!scale) {
    return Math.floor(speedHeatColors.length / 2);
  }

  if (scale.max <= scale.min) {
    return Math.floor(speedHeatColors.length / 2);
  }

  const normalized = Math.min(1, Math.max(0, (speed - scale.min) / (scale.max - scale.min)));
  return Math.min(speedHeatColors.length - 1, Math.floor(normalized * speedHeatColors.length));
}

function hideSpeedLegend() {
  elements.speedLegend.hidden = true;
}

function renderSpeedLegend(scale) {
  if (!scale) {
    hideSpeedLegend();
    return;
  }

  elements.speedLegend.hidden = false;
  elements.speedLegendCaption.textContent = 'Speed heatbar';
  elements.speedLegendPeak.textContent = `Peak ${formatNumber(scale.max, 1)} km/h`;
  elements.speedLegendMin.textContent = `${formatNumber(scale.min, 1)} km/h`;
  elements.speedLegendMid.textContent = `${formatNumber(scale.mid, 1)} km/h`;
  elements.speedLegendMax.textContent = `${formatNumber(scale.max, 1)} km/h`;
}

function buildTrackLayers(reportPoints) {
  const scale = speedScale(reportPoints);
  if (!scale) {
    return {
      layers: [
        L.polyline(reportPoints.map((point) => [point.latitude, point.longitude]), {
          color: '#0a8f7b',
          weight: 5,
          opacity: 0.88,
          lineJoin: 'round',
          lineCap: 'round',
        }),
      ],
      scale: null,
    };
  }

  const bucketedSegments = speedHeatColors.map(() => []);

  for (let index = 1; index < reportPoints.length; index += 1) {
    const previousPoint = reportPoints[index - 1];
    const currentPoint = reportPoints[index];
    const speed = segmentSpeedKph(previousPoint, currentPoint);

    if (speed === null) {
      continue;
    }

    const bucketIndex = speedColorIndex(speed, scale);
    bucketedSegments[bucketIndex].push([
      [previousPoint.latitude, previousPoint.longitude],
      [currentPoint.latitude, currentPoint.longitude],
    ]);
  }

  const layers = bucketedSegments
    .map((segments, index) => {
      if (!segments.length) {
        return null;
      }

      return L.polyline(segments, {
        color: speedHeatColors[index],
        weight: 6,
        opacity: 0.92,
        lineJoin: 'round',
        lineCap: 'round',
      });
    })
    .filter(Boolean);

  if (!layers.length) {
    layers.push(L.polyline(reportPoints.map((point) => [point.latitude, point.longitude]), {
      color: '#0a8f7b',
      weight: 5,
      opacity: 0.88,
      lineJoin: 'round',
      lineCap: 'round',
    }));
  }

  return { layers, scale };
}

function setBusy(isBusy) {
  elements.openLogButton.disabled = isBusy;
  elements.exportGeoJsonButton.disabled = isBusy || !state.currentFilePath;
  elements.exportKmlButton.disabled = isBusy || !state.currentFilePath;
}

function setStatus(message, badgeText = 'Ready') {
  elements.statusMessage.textContent = message;
  elements.mapBadge.textContent = badgeText;
}

function clearTrack() {
  if (state.trackLayer) {
    state.trackLayer.remove();
    state.trackLayer = null;
  }

  hideSpeedLegend();
}

function popupMarkup(title, point) {
  return `
    <div>
      <p class="popup-title">${title}</p>
      <p class="popup-meta">${formatCoordinatePair(point)}</p>
      <p class="popup-meta">${formatTimestamp(point.timestamp)}</p>
    </div>
  `;
}

function renderSentenceCounts(sentenceTypeCounts) {
  const rows = Object.entries(sentenceTypeCounts);
  elements.sentenceCounts.innerHTML = '';

  if (rows.length === 0) {
    elements.sentenceCounts.textContent = 'No valid NMEA sentences recovered.';
    return;
  }

  rows
    .sort((left, right) => right[1] - left[1])
    .forEach(([sentenceType, count]) => {
      const row = document.createElement('div');
      row.className = 'sentence-row';
      row.innerHTML = `<span>${sentenceType}</span><strong>${formatNumber(count)}</strong>`;
      elements.sentenceCounts.appendChild(row);
    });
}

function renderSummary(report) {
  elements.fixCount.textContent = formatNumber(report.fixCount);
  elements.distanceKm.textContent = `${formatNumber(report.distanceKilometers, 3)} km`;
  elements.sentenceCount.textContent = formatNumber(report.sentenceCount);
  elements.checksumCount.textContent = formatNumber(report.invalidChecksumCount);
  elements.rmcShare.textContent = report.sentenceCount
    ? `${formatNumber((report.fixCount / report.sentenceCount) * 100, 1)}% fixes`
    : '0% fixes';

  if (report.firstFix && report.lastFix) {
    elements.timeSpan.textContent = formatDuration(report.firstFix.timestamp, report.lastFix.timestamp);
    elements.startFix.textContent = `${formatCoordinatePair(report.firstFix)} at ${formatTimestamp(report.firstFix.timestamp)}`;
    elements.endFix.textContent = `${formatCoordinatePair(report.lastFix)} at ${formatTimestamp(report.lastFix.timestamp)}`;
  } else {
    elements.timeSpan.textContent = 'No fixes yet';
    elements.startFix.textContent = '-';
    elements.endFix.textContent = '-';
  }

  if (report.bounds) {
    elements.trackBounds.textContent = `${report.bounds.minLatitude.toFixed(4)} to ${report.bounds.maxLatitude.toFixed(4)} lat, ${report.bounds.minLongitude.toFixed(4)} to ${report.bounds.maxLongitude.toFixed(4)} lon`;
  } else {
    elements.trackBounds.textContent = '-';
  }

  const highestKph = topSpeedKph(report.points);
  elements.topSpeed.textContent = highestKph === null ? '-' : `${formatNumber(highestKph, 1)} km/h`;

  renderSentenceCounts(report.sentenceTypeCounts);
}

function renderMap(report) {
  clearTrack();

  if (!report.points.length) {
    map.setView(defaultCenter, defaultZoom);
    elements.mapOverlayMessage.textContent = 'The log parsed, but there were no valid position fixes to plot.';
    elements.mapOverlayMessage.hidden = false;
    elements.mapTitle.textContent = shortFileName(report.inputPath);
    setStatus('No valid fixes were found in the selected log.', 'No fixes');
    return;
  }

  elements.mapOverlayMessage.hidden = true;
  const { layers: trackLayers, scale } = buildTrackLayers(report.points);
  const startPoint = report.points[0];
  const endPoint = report.points[report.points.length - 1];

  const startMarker = L.circleMarker([startPoint.latitude, startPoint.longitude], {
    radius: 7,
    color: '#102522',
    weight: 2,
    fillColor: '#f16839',
    fillOpacity: 1,
  }).bindPopup(popupMarkup('Start', startPoint));

  const endMarker = L.circleMarker([endPoint.latitude, endPoint.longitude], {
    radius: 7,
    color: '#102522',
    weight: 2,
    fillColor: '#0a8f7b',
    fillOpacity: 1,
  }).bindPopup(popupMarkup('End', endPoint));

  state.trackLayer = L.featureGroup([...trackLayers, startMarker, endMarker]).addTo(map);
  map.fitBounds(state.trackLayer.getBounds(), {
    padding: [40, 40],
  });

  renderSpeedLegend(scale);
  elements.mapTitle.textContent = shortFileName(report.inputPath);
  setStatus(`Parsed ${formatNumber(report.fixCount)} fixes from ${shortFileName(report.inputPath)} with a speed heatbar overlay.`, 'Track ready');
}

function renderReport(report) {
  state.currentReport = report;
  elements.selectedFile.textContent = report.inputPath;
  renderSummary(report);
  renderMap(report);
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
    clearTrack();
    elements.mapOverlayMessage.hidden = false;
    elements.mapOverlayMessage.textContent = 'The parser failed for this file.';
    setStatus(error.message, 'Error');
  } finally {
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
      setStatus(`Export canceled.`, 'Ready');
    }
  } catch (error) {
    setStatus(error.message, 'Error');
  } finally {
    setBusy(false);
  }
}

elements.openLogButton.addEventListener('click', openLogFile);
elements.exportGeoJsonButton.addEventListener('click', () => exportTrack('geojson'));
elements.exportKmlButton.addEventListener('click', () => exportTrack('kml'));

setBusy(false);
renderSentenceCounts({});
