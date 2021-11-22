import { contextBridge, ipcRenderer } from 'electron'

export const APP_INFO_QUERY = 'app_info_query'
export type APP_INFO_TYPE = { appName: string; appVersion: string }
export const LOG_MSG = 'log_mmsg'
export type LOG_MSG_TYPE = {
  level: 'error' | 'warn' | 'info' | 'debug'
  message: string
}

export const AUDIO_START = 'audio_start'
export const AUDIO_STOP = 'audio_stop'
export const AUDIO_MSG = 'audio_msg'
export type AUDIO_MSG_TYPE = {
  error: string | undefined
  stderr: string
  stdout: string
}

if (ipcRenderer && contextBridge) {
  const preload = {
    PUBLIC_URL: process.env.PUBLIC_URL,
    PRINT_CALLSITE: !!process.env.ELECTRON_PRINT_CALLSITE,
    isTest: process.env.JEST_WORKER_ID !== undefined,
    isDev: process.env.NODE_ENV !== 'production',
    /**
     * Send the log message to the main thread
     */
    log: (data: LOG_MSG_TYPE) => {
      ipcRenderer.send(LOG_MSG, data)
    },
    app_info: ipcRenderer.sendSync(APP_INFO_QUERY) as APP_INFO_TYPE,
    audio: {
      start: () => ipcRenderer.send(AUDIO_START),
      stop: () => ipcRenderer.send(AUDIO_STOP),
      receive: (cb: (args: AUDIO_MSG_TYPE) => void) => {
        ipcRenderer.on(AUDIO_MSG, (_event, args) => cb(args as AUDIO_MSG_TYPE))
      },
    },
  }
  contextBridge.exposeInMainWorld('preloadApi', preload)
}
