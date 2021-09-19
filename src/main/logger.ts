import { LOG_MSG, LOG_MSG_TYPE } from '@/shared/constants'
import { ipcMain } from 'electron'
import winston, { format, transports } from 'winston'
import 'winston-daily-rotate-file'
const { timestamp, combine, printf, colorize, label } = format

/**
 * This module uses winston to write logs to the console and to a file.
 * It uses a separate file for logs coming from the main thread and the renderer thread.
 * Logs from the renderer are received through the ipc
 */

// TODO
// This is custom enough that it might be better to just do the file logging ourselves
// insted of pulling a big dependency like winston

const padding = Math.max(
  ...['error', 'warn', 'info'].map((level) => level.length)
)

const padEndLevel = format((info) => {
  info.level = info.level.padEnd(padding).toUpperCase()
  return info
})

const fileFormat = combine(
  timestamp(),
  padEndLevel(),
  printf((info) => `${info.timestamp} ${info.level} ${info.message}`)
)

const consoleFormat = combine(
  timestamp({ format: 'HH:mm:ss' }),
  padEndLevel(),
  colorize({ level: true }),
  printf(
    (info) => `[${info.label}] ${info.timestamp} ${info.level} ${info.message}`
  )
)

const levels = {
  levels: {
    error: 0,
    warn: 1,
    info: 2,
    trace: 3,
    debug: 4,
  },
  colors: {
    error: 'red',
    warn: 'yellow',
    info: 'green',
    trace: 'gray',
    debug: 'blue',
  },
}

winston.addColors(levels.colors)

const fileTransportOptions = {
  dirname: 'logs/%DATE%',
  datePattern: 'YYYY-MM-DD',
}

const logger = winston.createLogger({
  level: 'debug',
  levels: levels.levels,
  format: fileFormat,
  transports: [
    new transports.DailyRotateFile({
      ...fileTransportOptions,
      filename: 'main.log',
    }),
  ],
})
export const log = logger

const rendererLogger = winston.createLogger({
  level: 'debug',
  levels: levels.levels,
  format: fileFormat,
  transports: [
    new transports.DailyRotateFile({
      ...fileTransportOptions,
      filename: 'renderer.log',
    }),
  ],
})

if (process.env.NODE_ENV !== 'production') {
  logger.add(
    new transports.Console({
      format: combine(label({ label: 'main' }), consoleFormat),
      eol: ' ',
    })
  )
  rendererLogger.add(
    new transports.Console({
      format: combine(label({ label: 'renderer' }), consoleFormat),
      eol: ' ',
    })
  )
}

ipcMain.on(LOG_MSG, (_event, data: LOG_MSG_TYPE) => {
  rendererLogger.log(data.level, data.message)
})
