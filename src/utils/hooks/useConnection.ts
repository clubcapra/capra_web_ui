import { useInterval } from 'utils/hooks/useInterval'
import { useState } from 'react';

export const useConnection = () => {
  // @ts-ignore
  const { connection } = navigator

  const [state, setState] = useState({
    rtt: connection.rtt,
    type: connection.type,
    effectiveType: connection.effectiveType,
  })

  useInterval(() => {
    //@ts-ignore
    const { connection } = navigator

    setState({
      rtt: connection.rtt,
      type: connection.type,
      effectiveType: connection.effectiveType,
    })
  }, 1000)

  return { ...state }
}
