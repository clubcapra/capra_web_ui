import { log } from '@/main/logger'
import {
  AUDIO_MSG,
  AUDIO_MSG_TYPE,
  AUDIO_START,
  AUDIO_STOP,
} from '@/main/preload'
import { ipcMain, IpcMainEvent } from 'electron'
import { execa, ExecaChildProcess } from 'execa'

// This is a pretty hacky solution to have 2 way audio
// Essentially we don't use electron and let ros handle it.
// We simply start the node in a child process

let audio_process: ExecaChildProcess | null = null

ipcMain.on(AUDIO_START, (event) => {
  if (audio_process !== null) {
    log.warn('audio_process is already running')
    return
  }
  log.info('starting audio process')
  // WARN this should be configurable
  if (process.platform === 'linux') {
    void (async () => {
      audio_process = execa('roslaunch', ['capra_audio', 'audio_ui.launch'])
      audio_process.stdout?.pipe(process.stdout)
      audio_process.stderr?.pipe(process.stderr)
      void audio_process.on('spawn', () => log.info('audio_process spawned'))
      void audio_process.on('close', () => log.info('audio_process closed'))
      try {
        await audio_process
      } catch (error) {
        log.error(error)
      }
    })()
  } else if (process.platform === 'win32') {
    // TODO figure out a way to use wsl to launch capra_audio
    sendNotSupportedMessage(event, 'Windows')
  } else {
    sendNotSupportedMessage(event, process.platform)
  }
})

ipcMain.on(AUDIO_STOP, () => {
  if (audio_process === null) {
    log.warn('Trying to stop audio_process but it is not running')
    return
  }
  if (!audio_process.pid) {
    log.warn('audio_process.pid is not valid')
    return
  }

  log.info('sending audio_process kill signal')

  audio_process.kill()
  audio_process = null
})

function sendNotSupportedMessage(event: IpcMainEvent, platformName: string) {
  const message = `Audio streaming is currently not supported on ${platformName}`
  log.error(message)
  const reply: AUDIO_MSG_TYPE = {
    error: message,
    stderr: '',
    stdout: '',
  }
  event.reply(AUDIO_MSG, reply)
}
