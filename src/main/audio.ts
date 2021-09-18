import { AUDIO_MSG, AUDIO_MSG_TYPE, AUDIO_START } from '@/shared/constants'
import { exec } from 'child_process'
import electron from 'electron'

const { ipcMain } = electron

// This is a pretty hacky solution to have 2 way audio
// Essentially we don't use electron and let ros handle it.
// We simply start the node in a child process

ipcMain.on(AUDIO_START, (event) => {
  // WARN this should be configurable
  const launch = 'roslaunch capra_audio audio_ui.launch'
  if (process.platform === 'linux') {
    exec(launch, (error, stdout, stderr) => {
      const reply: AUDIO_MSG_TYPE = {
        error: error?.message,
        stderr,
        stdout,
      }
      event.reply(AUDIO_MSG, reply)
      // eslint-disable-next-line no-console
      console.log(stdout)
      console.error(stderr)
    })
  } else if (process.platform === 'win32') {
    // TODO figure out a way to use wsl to launch capra_audio
    sendNotSupportedMessage(event, 'Windows')
  } else {
    sendNotSupportedMessage(event, process.platform)
  }
})

function sendNotSupportedMessage(
  event: electron.IpcMainEvent,
  platformName: string
) {
  const message = `Audio streaming is currently not supported on ${platformName}`
  console.error(message)
  const reply: AUDIO_MSG_TYPE = {
    error: message,
    stderr: '',
    stdout: '',
  }
  event.reply(AUDIO_MSG, reply)
}