import { useEffect, useRef } from 'react'

/**
 * Hook wrapper around setInterval() that will automatically
 * stop the interval when the component unmount
 *
 * @param callback will be called after every interval
 * @param delay time between interval in ms
 */
export const useInterval = (callback: Function, delay: number): void => {
  const savedCallback = useRef<Function>()

  useEffect(() => {
    savedCallback.current = callback
  }, [callback])

  useEffect(() => {
    const tick = (): void => {
      if (savedCallback.current) savedCallback.current()
    }

    if (delay !== null) {
      const id = setInterval(tick, delay)
      return () => clearInterval(id)
    }
  }, [delay])
}
