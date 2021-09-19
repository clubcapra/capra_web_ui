import '@/main/audio'
import { log } from '@/main/logger'
import { APP_INFO, APP_INFO_ARG, APP_INFO_QUERY } from '@/shared/constants'
import { app, BrowserWindow, ipcMain, powerSaveBlocker } from 'electron'
import installExtension, {
  REACT_DEVELOPER_TOOLS,
  REDUX_DEVTOOLS,
} from 'electron-devtools-installer'
import path from 'path'
import { isDev } from './isDev'

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
    log.info('Window created')
    mainWindow
      .loadURL(getAssetURL('index.html'))
      .then(() => {
        log.info('index.html loaded')
      })
      .catch(log.error)

    if (isDev) {
      // This is to make sure the devtools only opens when extensions are loaded
      mainWindow.once('ready-to-show', () => {
        mainWindow?.webContents.openDevTools()
      })
    } else {
      mainWindow.removeMenu()
    }

    mainWindow.on('closed', () => {
      log.info('window closed')
      mainWindow = null
    })
  }
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
        log.error('Error when installing devtools extension', err)
      }
    }
  })
  .then(() => {
    createWindow()
    app.on('activate', () => {
      log.info('window activated')
    })
  })
  .catch((err) => log.error(err))

ipcMain.on(APP_INFO_QUERY, (event) => {
  event.reply(APP_INFO, {
    appName: app.name,
    appVersion: app.getVersion(),
  } as APP_INFO_ARG)
})

// Since we don't ever want the display to sleep while the robot is connected
// we try to force it to never sleep
const id = powerSaveBlocker.start('prevent-display-sleep')
log.info(`prevent-display-sleep started: ${powerSaveBlocker.isStarted(id)}`)
