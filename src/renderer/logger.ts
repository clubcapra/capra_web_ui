// The console api uses any so it's simpler to allow it
/* eslint-disable @typescript-eslint/no-explicit-any */

// This file is configuring the console so it's easier to allow it here
/* eslint-disable no-console */

import { LOG_MSG, LOG_MSG_TYPE } from '@/shared/constants'
const { ipcRenderer } = window.require('electron')
import { format } from 'date-fns'

/**
 * This is a custom logger that should be used in place of console.log
 * It will add more information to the logs. More importantly, it will send an event
 * to electron main to write the logs to a log file.
 *
 * By default, the log callsite is not shown properly by the devtool.
 * If you need the precise callsite, you can set ELECTRON_PRINT_CALLSITE to true in .env.
 * This will still show the wrong callsite, but it will append the appropriate callsite
 * with filename and linenumber to the log message
 *
 * Example usage:
 *
 * import { log } from '@/renderer/logger'
 *
 * log.error('This is an error message')
 * log.warn('This is an warn message')
 * log.info('This is an info message')
 * log.trace('This is a trace message')
 * log.debug('This is a debug message')
 */

// TODO
// maybe support some kind of module specifier.
// Unfortunately that info is not available in js so it would need to be customised in each file

type Levels = 'error' | 'warn' | 'info' | 'debug'

const timeFormat = (date: Date) => format(date, '[HH:mm:ss]')
const logFormat = (level: Levels) =>
  `${timeFormat(new Date())} ${level.toUpperCase()} %s`

/**
 *  Sends the log to the main thread to be written to a file
 */
function sendLog(level: Levels, args: any[]) {
  const data: LOG_MSG_TYPE = {
    level,
    // winston doesn't support multi args so it's easier to do this here
    message: args.join(' '),
  }
  ipcRenderer.send(LOG_MSG, data)
}

function callsites() {
  const _prepareStackTrace = Error.prepareStackTrace
  Error.prepareStackTrace = (_, stack) => stack
  // eslint-disable-next-line @typescript-eslint/ban-ts-comment
  // @ts-ignore
  const stack = new Error().stack.slice(1)
  Error.prepareStackTrace = _prepareStackTrace
  // eslint-disable-next-line @typescript-eslint/ban-ts-comment
  // @ts-ignore
  return stack as NodeJS.CallSite[]
}

const printCallsite =
  process.env.ELECTRON_PRINT_CALLSITE && process.env.NODE_ENV !== 'production'
function callsite() {
  // WARN
  // This number is equivalent to the number of functions called
  // to get to the actual console.log
  const callsite = callsites()[3]
  return printCallsite && callsite
    ? `\n${callsite.getFileName()}:${callsite.getLineNumber()}`
    : ''
}

function logFn(level: Levels, args: any[]) {
  sendLog(level, args)
  console[level](logFormat(level), ...args, callsite())
}

export const error = (...args: any[]) => logFn('error', args)
export const warn = (...args: any[]) => logFn('warn', args)
export const info = (...args: any[]) => logFn('info', args)
export const debug = (...args: any[]) => logFn('debug', args)

export const log = {
  error,
  warn,
  info,
  debug,
}
