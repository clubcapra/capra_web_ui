const electron = require('electron')
const { app, BrowserWindow, ipcMain } = electron
const path = require('path')
const url = require('url')
const isDev = require('./isDev')
const { channels } = require('./shared/constants')

// WARNING!
// DO NOT USE electron-devtools-installer
// if you did and the app doesn't show anymore,
// delete %AppData%/electron and %AppData%/[project name]
// see https://stackoverflow.com/questions/57614066/electron-app-onready-never-being-called-and-electron-window-never-showing

let mainWindow

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 900,
    height: 680,
    icon: path.join(__dirname, '../../public/logo512.png'),
    title: 'capra_web_ui',
    webPreferences: {
      nodeIntegration: true,
    },
  })

  mainWindow.loadURL(
    isDev
      ? 'http://localhost:3000'
      : url.format({
          pathname: path.join(__dirname, '../../build/index.html'),
          protocol: 'file:',
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

ipcMain.on(channels.APP_INFO, event => {
  event.sender.send(channels.APP_INFO, {
    appName: app.getName(),
    appVersion: app.getVersion(),
  })
})
