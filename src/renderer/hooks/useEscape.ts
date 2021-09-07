import { useKeyPress } from '@/renderer/hooks/useKeyPressed'
import { useEffect } from 'react'

export const useEscape = (callback: () => void) => {
  const isEscapePressed = useKeyPress('Escape')
  useEffect(() => {
    if (isEscapePressed) {
      callback()
    }
  }, [isEscapePressed])
}
