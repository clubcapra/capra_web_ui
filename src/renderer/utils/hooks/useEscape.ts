import { useKeyPress } from '@/renderer/utils/hooks/useKeyPressed'
import { useEffect } from 'react'

export const useEscape = (callback: () => void) => {
  const isEscapePressed = useKeyPress('Escape')
  useEffect(() => {
    if (isEscapePressed) {
      callback()
    }
  }, [isEscapePressed])
}
