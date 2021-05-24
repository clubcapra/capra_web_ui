import React from 'react'
import { AUDIO_MSG, AUDIO_MSG_TYPE, AUDIO_START } from '@/shared/constants'
const { ipcRenderer } = window.require('electron')

export const AudioStart = () => {
  const startAudio = () => {
    ipcRenderer.send(AUDIO_START)
  }

  return <button onClick={startAudio}>Start Audio</button>
}

ipcRenderer.on(
  AUDIO_MSG,
  (_event, { error, stderr, stdout }: AUDIO_MSG_TYPE) => {
    if (stdout) {
      // eslint-disable-next-line no-console
      console.log(stdout)
    }
    if (error) {
      console.error(error)
    }
    if (stderr) {
      console.error(stderr)
    }
  }
)
