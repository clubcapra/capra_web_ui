import electron from 'electron'
import path from 'path'
import { isDev } from './isDev'
import { APP_INFO, APP_INFO_ARG, APP_INFO_QUERY } from '../shared/constants'
import installExtension, {
  REACT_DEVELOPER_TOOLS,
  REDUX_DEVTOOLS,
} from 'electron-devtools-installer'
import { powerSaveBlocker } from 'electron'
import { initTerminalClient } from './terminalClient'

const { app, BrowserWindow, ipcMain } = electron

let mainWindow: Electron.BrowserWindow | null = null

const getAssetURL = (asset: string) => {
  if (isDev) {
    const port = process.env.ELECTRON_SNOWPACK_PORT
    if (!port) {
      throw Error('port is not a string')
    }
    return new URL(asset, `http://localhost:${port}/`).toString()
  } else {
    return new URL(`file:///${path.join(__dirname, asset)}`).href
  }
}

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1280,
    height: 720,
    title: 'capra_web_ui',
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false, // WARN this is really important if we want to use ipc
      webSecurity: false, // Allow CORS for robot_description server
    },
  })

  if (mainWindow) {
    mainWindow.loadURL(getAssetURL('index.html')).catch(console.error)

    if (isDev) {
      // This is to make sure the devtools only opens when extensions are loaded
      mainWindow.once('ready-to-show', () => {
        mainWindow?.webContents.openDevTools()
      })
    } else {
      mainWindow.removeMenu()
    }

    mainWindow.on('closed', () => (mainWindow = null))
  }
  initTerminalClient()
}

app
  .whenReady()
  .then(async () => {
    // WARNING!
    // If you use electron-devtools-installer and the app doesn't show anymore,
    // delete %AppData%/electron and %AppData%/[project name]
    // see https://stackoverflow.com/questions/57614066/electron-app-onready-never-being-called-and-electron-window-never-showing

    if (isDev) {
      try {
        await installExtension(REACT_DEVELOPER_TOOLS, {
          loadExtensionOptions: { allowFileAccess: true },
          forceDownload: false,
        })
        await installExtension(REDUX_DEVTOOLS, {
          loadExtensionOptions: { allowFileAccess: true },
          forceDownload: false,
        })
      } catch (err) {
        console.error('Error when installing devtools extension', err)
      }
    }
  })
  .then(() => {
    createWindow()
    app.on('activate', () => {
      // On macOS it's common to re-create a window in the app when the
      // dock icon is clicked and there are no other windows open.
      if (BrowserWindow.getAllWindows().length === 0) {
        createWindow()
      }
    })
  })
  .catch((err) => console.error(err))

// Quit when all windows are closed, except on macOS. There, it's common
// for applications and their menu bar to stay active until the user quits
// explicitly with Cmd + Q.
app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit()
  }
})

ipcMain.on(APP_INFO_QUERY, (event) => {
  event.reply(APP_INFO, {
    appName: app.name,
    appVersion: app.getVersion(),
  } as APP_INFO_ARG)
})

// Since we don't every want the display to sleep while the robot is connected
// we try to force it to never sleep
const id = powerSaveBlocker.start('prevent-display-sleep')
// eslint-disable-next-line no-console
console.log('prevent-display-sleep started: ', powerSaveBlocker.isStarted(id))
