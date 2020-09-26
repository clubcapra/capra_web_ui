import React from 'react'
import ReactDOM from 'react-dom'
import App from './App'
import { channels } from '~../shared/constants'
const { ipcRenderer } = window.require('electron')

ReactDOM.render(<App />, document.getElementById('root'))

ipcRenderer.send(channels.APP_INFO)
ipcRenderer.on(channels.APP_INFO, (_event, arg) => {
  ipcRenderer.removeAllListeners(channels.APP_INFO)
  const { appName, appVersion } = arg
  // eslint-disable-next-line no-console
  console.log(appName, appVersion)
})
