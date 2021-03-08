import { format } from 'date-fns'
import { useInterval } from '@/renderer/utils/hooks/useInterval'
import React, { FC, useState } from 'react'

const timeFormat = (date: Date) => format(date, 'HH:mm:ss')

export const TimeDisplay: FC = () => {
  const [time, setTime] = useState(timeFormat(new Date()))

  useInterval(() => {
    setTime(timeFormat(new Date()))
  }, 1000)

  return <div>{time}</div>
}
