import { useEffect, useRef } from 'react'

export const useHasChanged = <T>(val: T) => {
  const prevVal = usePrevious(val)
  return prevVal !== val
}

export const usePrevious = <T>(value: T) => {
  const ref = useRef<T>()
  useEffect(() => {
    ref.current = value
  })
  return ref.current
}
