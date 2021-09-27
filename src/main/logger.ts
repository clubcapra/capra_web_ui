import { isDev } from '@/main/isDev'
import { LOG_MSG, LOG_MSG_TYPE } from '@/shared/constants'
import { app, ipcMain } from 'electron'
import electronLog, { LogMessage, LogLevel } from 'electron-log'
import path from 'path'
import datefns from 'date-fns'
import chalk from 'chalk'
chalk.level = 3

/**
 * This module uses winston to write logs to the console and to a file.
 * It uses a separate file for logs coming from the main thread and the renderer thread.
 * Logs from the renderer are received through the ipc
 */

const padding = Math.max(
  ...['error', 'warn', 'info', 'debug'].map((level) => level.length)
)

function formatLevel(level: LogLevel) {
  const levelString = level.toUpperCase().padEnd(padding)
  switch (level) {
    case 'error':
      return chalk.red(levelString)
    case 'warn':
      return chalk.yellow(levelString)
    case 'info':
      return chalk.green(levelString)
    case 'debug':
      return chalk.blue(levelString)
    default:
      return levelString
  }
}

function consoleFormat(message: LogMessage) {
  const date = datefns.format(message.date, 'HH:mm:ss')
  return `${date} ${formatLevel(message.level)} ${message.data}`
}

function fileFormat(message: LogMessage) {
  return `${message.date.toISOString()} ${message.level
    .toUpperCase()
    .padEnd(padding)} ${message.data}`
}

function resolvePath(filename: string) {
  return path.join(
    isDev ? process.cwd() : path.join(app.getPath('appData'), app.name),
    'logs',
    date,
    filename
  )
}

const rendererElectronLog = electronLog.create('renderer')
rendererElectronLog.transports.file.resolvePath = () =>
  resolvePath('renderer.log')
rendererElectronLog.transports.file.format = fileFormat
rendererElectronLog.transports.console.format = consoleFormat

ipcMain.on(LOG_MSG, (_event, data: LOG_MSG_TYPE) => {
  rendererElectronLog[data.level](data.message)
})

const mainElectronLog = electronLog.create('main')
const date = datefns.format(new Date(), 'yyyy-MM-dd')
mainElectronLog.transports.file.resolvePath = () => resolvePath('main.log')
mainElectronLog.transports.file.format = fileFormat
mainElectronLog.transports.console.format = consoleFormat

export const log = mainElectronLog

log.error('this is an error')
log.warn('this is a warn')
log.info('this is an info')
log.debug('this is a debug')
