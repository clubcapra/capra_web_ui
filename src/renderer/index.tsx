import React from 'react'
import ReactDOM from 'react-dom'
import './index.css'
import App from './App'
import * as serviceWorker from './serviceWorker'
import { channels } from '~../shared/constants'
// import electron from 'electron'

ReactDOM.render(<App />, document.getElementById('root'))

if (window.require) {
  const { ipcRenderer } = window.require('electron')
  ipcRenderer.send(channels.APP_INFO)
  ipcRenderer.on(channels.APP_INFO, (_event, arg) => {
    ipcRenderer.removeAllListeners(channels.APP_INFO)
    const { appName, appVersion } = arg
    // eslint-disable-next-line no-console
    console.log(appName, appVersion)
  })
}

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister()
