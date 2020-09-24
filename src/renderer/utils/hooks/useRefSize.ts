import { useState, useLayoutEffect } from 'react'
export function useRefSize(ref: React.RefObject<HTMLElement>): number[] {
  const [size, setSize] = useState([0, 0])
  useLayoutEffect(() => {
    function updateSize() {
      if (ref?.current) {
        setSize([ref.current?.clientWidth, ref.current.clientHeight])
      }
    }
    window.addEventListener('resize', updateSize)
    updateSize()
    return () => window.removeEventListener('resize', updateSize)
  }, [ref])
  return size
}
