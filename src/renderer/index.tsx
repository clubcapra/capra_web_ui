import { info } from '@/renderer/logger'
import { APP_INFO, APP_INFO_ARG, APP_INFO_QUERY } from '@/shared/constants'
import React from 'react'
import ReactDOM from 'react-dom'
import { App } from './App'
const { ipcRenderer } = window.require('electron')

ReactDOM.render(<App />, document.getElementById('root'))

ipcRenderer.send(APP_INFO_QUERY)
ipcRenderer.on(APP_INFO, (_event, arg: APP_INFO_ARG) => {
  ipcRenderer.removeAllListeners(APP_INFO)
  const { appName, appVersion } = arg
  info(appName, appVersion)
})
