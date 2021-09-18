import { AUDIO_MSG, AUDIO_MSG_TYPE, AUDIO_START } from '@/shared/constants'
import { useEffect } from 'react'
import { toast } from 'react-toastify'
const { ipcRenderer } = window.require('electron')

export const useAudio = () => {
  useEffect(() => {
    ipcRenderer.on(AUDIO_MSG, (_event, { error }: AUDIO_MSG_TYPE) => {
      if (error) {
        toast.error(error)
      }
    })
  }, [])
  const start = () => {
    ipcRenderer.send(AUDIO_START)
  }
  return [start]
}
