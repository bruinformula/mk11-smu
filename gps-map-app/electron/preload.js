const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('gpsMapApp', {
  pickLogFile: () => ipcRenderer.invoke('dialog:pick-log-file'),
  parseLogFile: (filePath) => ipcRenderer.invoke('parser:parse-log-file', filePath),
  exportTrack: (filePath, format) => ipcRenderer.invoke('parser:export-track', {
    filePath,
    format,
  }),
});
