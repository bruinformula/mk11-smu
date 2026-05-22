const { app, BrowserWindow, dialog, ipcMain } = require('electron');
const { spawn } = require('node:child_process');
const path = require('node:path');

const APP_ROOT = path.join(__dirname, '..');
const PARSER_PATH = path.join(APP_ROOT, 'python', 'nmea_parser.py');

function createWindow() {
  const window = new BrowserWindow({
    width: 1480,
    height: 920,
    minWidth: 1100,
    minHeight: 760,
    backgroundColor: '#102322',
    autoHideMenuBar: true,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });

  window.loadFile(path.join(APP_ROOT, 'src', 'index.html'));
}

function pythonLaunchPlan() {
  if (process.env.GPS_MAP_PYTHON) {
    return [{ command: process.env.GPS_MAP_PYTHON, prefixArgs: [] }];
  }

  if (process.platform === 'win32') {
    return [
      { command: 'py', prefixArgs: ['-3'] },
      { command: 'python', prefixArgs: [] },
      { command: 'python3', prefixArgs: [] },
    ];
  }

  return [
    { command: 'python3', prefixArgs: [] },
    { command: 'python', prefixArgs: [] },
  ];
}

function runParser(parserArgs) {
  const launchAttempts = pythonLaunchPlan();

  return new Promise((resolve, reject) => {
    const tryLaunch = (index) => {
      if (index >= launchAttempts.length) {
        reject(new Error('Unable to find a working Python runtime. Set GPS_MAP_PYTHON if needed.'));
        return;
      }

      const attempt = launchAttempts[index];
      const commandArgs = [...attempt.prefixArgs, PARSER_PATH, ...parserArgs];
      const child = spawn(attempt.command, commandArgs, {
        cwd: APP_ROOT,
      });

      let stdout = '';
      let stderr = '';
      let launched = false;

      child.stdout.on('data', (chunk) => {
        launched = true;
        stdout += chunk.toString();
      });

      child.stderr.on('data', (chunk) => {
        launched = true;
        stderr += chunk.toString();
      });

      child.on('error', (error) => {
        if (error.code === 'ENOENT') {
          tryLaunch(index + 1);
          return;
        }

        reject(error);
      });

      child.on('close', (code) => {
        if (!launched && code !== 0) {
          tryLaunch(index + 1);
          return;
        }

        if (code !== 0) {
          reject(new Error(stderr.trim() || `Parser exited with code ${code}`));
          return;
        }

        try {
          resolve(JSON.parse(stdout));
        } catch (error) {
          reject(new Error(`Parser returned invalid JSON: ${error.message}`));
        }
      });
    };

    tryLaunch(0);
  });
}

ipcMain.handle('dialog:pick-log-file', async () => {
  const result = await dialog.showOpenDialog({
    title: 'Open GNSS log',
    properties: ['openFile'],
    filters: [
      { name: 'Log files', extensions: ['log', 'txt', 'nmea'] },
      { name: 'All files', extensions: ['*'] },
    ],
  });

  if (result.canceled || result.filePaths.length === 0) {
    return null;
  }

  return result.filePaths[0];
});

ipcMain.handle('parser:parse-log-file', async (_event, filePath) => runParser([filePath]));

ipcMain.handle('parser:export-track', async (_event, { filePath, format }) => {
  const extension = format === 'geojson' ? 'geojson' : 'kml';
  const parsedPath = path.parse(filePath);
  const saveResult = await dialog.showSaveDialog({
    title: `Export ${format.toUpperCase()} track`,
    defaultPath: path.join(parsedPath.dir, `${parsedPath.name}.${extension}`),
    filters: [{ name: extension.toUpperCase(), extensions: [extension] }],
  });

  if (saveResult.canceled || !saveResult.filePath) {
    return null;
  }

  const parserArgs = [filePath];
  if (format === 'geojson') {
    parserArgs.push('--geojson-output', saveResult.filePath);
  } else {
    parserArgs.push('--kml-output', saveResult.filePath);
  }

  await runParser(parserArgs);
  return saveResult.filePath;
});

app.whenReady().then(() => {
  createWindow();

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow();
    }
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
