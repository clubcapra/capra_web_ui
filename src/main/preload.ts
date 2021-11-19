import {
  AUDIO_MSG,
  AUDIO_MSG_TYPE,
  AUDIO_START,
  AUDIO_STOP,
} from '@/main/audio'
import {
  APP_INFO_QUERY,
  APP_INFO_TYPE,
  LOG_MSG,
  LOG_MSG_TYPE,
} from '@/shared/constants'
import { contextBridge, ipcRenderer } from 'electron'

export const preload = {
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
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    receive: (cb: (args: AUDIO_MSG_TYPE) => void) => {
      ipcRenderer.on(AUDIO_MSG, (_event, args) => cb(args as AUDIO_MSG_TYPE))
    },
  },
}

contextBridge.exposeInMainWorld('preloadApi', preload)
