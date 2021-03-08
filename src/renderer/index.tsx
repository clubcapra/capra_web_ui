import React from 'react'
import ReactDOM from 'react-dom'
import App from './App'
import { channels } from '@/shared/constants'

const { ipcRenderer } = window.require('electron')

console.trace('mounting app...')

ReactDOM.render(<App />, document.getElementById('root'))

console.trace('App mounted')

ipcRenderer.send(channels.APP_INFO)
ipcRenderer.on(channels.APP_INFO, (_event, arg) => {
  ipcRenderer.removeAllListeners(channels.APP_INFO)
  const { appName, appVersion } = arg
  // eslint-disable-next-line no-console
  console.log(appName, appVersion)
})
