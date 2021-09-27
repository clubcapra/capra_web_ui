import { useEffect, useRef } from 'react'

export const useIsMounted = function useIsMounted() {
  const isMounted = useRef(false)
  useEffect(function setIsMounted() {
    isMounted.current = true
    return () => {
      isMounted.current = false
    }
  }, [])

  return isMounted
}
