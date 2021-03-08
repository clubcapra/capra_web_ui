import electron from 'electron'
import path from 'path'
import { isDev } from './isDev'
import { channels } from '../shared/constants'
import url from 'url'

const { app, BrowserWindow, ipcMain } = electron

// WARNING!
// DO NOT USE electron-devtools-installer
// if you did and the app doesn't show anymore,
// delete %AppData%/electron and %AppData%/[project name]
// see https://stackoverflow.com/questions/57614066/electron-app-onready-never-being-called-and-electron-window-never-showing

let mainWindow: Electron.BrowserWindow | null = null

app.allowRendererProcessReuse = true

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
    mainWindow.loadURL(
      isDev
        ? new URL(
            'index.html',
            `http://localhost:${process.env.DEV_PORT}/`
          ).toString()
        : url.format({
            pathname: path.join(__dirname, 'index.html'),
            protocol: 'file',
            slashes: true,
          })
    )

    if (isDev) {
      // TODO add react + redux devtools
      mainWindow.webContents.openDevTools()
    } else {
      mainWindow.removeMenu()
    }

    mainWindow.on('closed', () => (mainWindow = null))
  }
}

app.on('ready', () => {
  createWindow()
})

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit()
  }
})

app.on('activate', () => {
  if (mainWindow === null) {
    createWindow()
  }
})

ipcMain.on(channels.APP_INFO, (event) => {
  event.sender.send(channels.APP_INFO, {
    appName: app.name,
    appVersion: app.getVersion(),
  })
})
