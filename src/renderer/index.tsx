import { log } from '@/renderer/logger'
import React from 'react'
import ReactDOM from 'react-dom'
import { App } from './App'

ReactDOM.render(<App />, document.getElementById('root'))

log.info(
  'App finished loading... appVersion: ',
  window.preloadApi.app_info.appVersion
)
