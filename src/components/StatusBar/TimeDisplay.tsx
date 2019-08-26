// import { format } from 'date-fns'
import format from 'date-fns/fp/format'
import { useInterval } from 'utils/hooks/useInterval'
import React, { FC, useState } from 'react'

const timeFormat = format('HH:mm:ss')

export const TimeDisplay: FC = () => {
  const [time, setTime] = useState(timeFormat(new Date()))

  useInterval(() => {
    setTime(timeFormat(new Date()))
  }, 1000)

  return <div>{time}</div>
}
