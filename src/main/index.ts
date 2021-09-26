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

const appVersion = isDev ? process.env.npm_package_version : app.getVersion()

function createWindow() {
  const mainWindow = new BrowserWindow({
    width: 1280,
    height: 720,
    title: `Capra UI v${appVersion}`,
    webPreferences: {
      nodeIntegration: true,
      // TODO https://www.electronjs.org/docs/tutorial/context-isolation
      contextIsolation: false, // WARN this is really important if we want to use ipc
      webSecurity: false, // Allow CORS for robot_description server
    },
  })
  log.info('Window created')

  mainWindow
    .loadURL(getAssetURL('index.html'))
    .then(() => {
      log.info('index.html loaded')
    })
    .catch(log.error)

  if (isDev) {
    // This is to make sure the devtools only opens when extensions are loaded
    log.info('opening devtools')
    mainWindow?.webContents.openDevTools()
  } else {
    // mainWindow.removeMenu()
  }

  mainWindow.once('ready-to-show', () => {
    log.info('ready to show')
  })

  mainWindow.on('closed', () => {
    log.info('Main window closed')
  })
}

app
  .whenReady()
  .then(async () => {
    log.info('app is ready')

    // Since we don't ever want the display to sleep while the robot is connected
    // we try to force it to never sleep
    const id = powerSaveBlocker.start('prevent-display-sleep')
    log.info(`prevent-display-sleep started: ${powerSaveBlocker.isStarted(id)}`)

    // WARNING!
    // If you use electron-devtools-installer and the app doesn't show anymore,
    // delete %AppData%/electron and %AppData%/[project name]
    // see https://stackoverflow.com/questions/57614066/electron-app-onready-never-being-called-and-electron-window-never-showing

    if (isDev) {
      log.info('Loading devtools extensions')
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

    // only create the window once extensions are loaded
    createWindow()

    app.on('activate', () => {
      log.info('window activated')
    })
  })
  .catch((err) => log.error(err))

app.on('window-all-closed', () => {
  log.info('All window closed, quitting...')
  app.quit()
})

ipcMain.on(APP_INFO_QUERY, (event) => {
  event.reply(APP_INFO, {
    appName: app.name,
    appVersion: appVersion,
  } as APP_INFO_ARG)
})
