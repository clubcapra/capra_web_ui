import { useEffect, useState, FC } from 'react'

interface Props {
  value: any
  delay: number
}

export const useDebounce: FC<Props> = ({ value, delay }) => {
  const [debouncedValue, setDebouncedValue] = useState(value)

  useEffect(() => {
    const timeoutId = setTimeout(() => {
      setDebouncedValue(value)
    }, delay)

    return () => clearTimeout(timeoutId)
  }, [delay, value])

  return debouncedValue
}
