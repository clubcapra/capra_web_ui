import React from 'react'
import ReactDOM from 'react-dom'
import { App } from './App'
import { APP_INFO, APP_INFO_ARG, APP_INFO_QUERY } from '@/shared/constants'
const { ipcRenderer } = window.require('electron')

ReactDOM.render(<App />, document.getElementById('root'))

ipcRenderer.send(APP_INFO_QUERY)
ipcRenderer.on(APP_INFO, (_event, arg: APP_INFO_ARG) => {
  ipcRenderer.removeAllListeners(APP_INFO)
  const { appName, appVersion } = arg
  // eslint-disable-next-line no-console
  console.log(appName, appVersion)
})
