export const APP_INFO = 'app_info'
export type APP_INFO_TYPE = { appName: string; appVersion: string }
export const APP_INFO_QUERY = 'app_info_query'

export const LOG_MSG = 'log_msg'
export type LOG_MSG_TYPE = {
  level: 'error' | 'warn' | 'info' | 'debug'
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  message: string
}
