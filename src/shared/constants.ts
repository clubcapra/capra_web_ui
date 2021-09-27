export const APP_INFO = 'app_info'
export type APP_INFO_ARG = { appName: string; appVersion: string }
export const APP_INFO_QUERY = 'app_info_query'

export const AUDIO_START = 'audio_start'
export const AUDIO_STOP = 'audio_stop'
export const AUDIO_MSG = 'audio_msg'
export type AUDIO_MSG_TYPE = {
  error: string | undefined
  stderr: string
  stdout: string
}

export const LOG_MSG = 'log_msg'
export type LOG_MSG_TYPE = {
  level: 'error' | 'warn' | 'info' | 'debug'
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  message: string
}
