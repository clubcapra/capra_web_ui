import React, { FC, useState } from 'react'
import {
  MdBluetoothConnected,
  MdBluetoothDisabled,
  MdSignalCellular4Bar,
  MdSignalCellularOff,
  MdSignalWifi1Bar,
  MdSignalWifi2Bar,
  MdSignalWifi3Bar,
  MdSignalWifi4Bar,
  MdSignalWifiOff,
  MdSettingsEthernet,
} from 'react-icons/md'

const useConnection = () => {
  // const connection: { rtt: number; type: string; effectiveType: string } =
  //   navigator['connection']

  const [state] = useState({
    rtt: 0,
    type: 'wifi',
    effectiveType: '4g',
  })

  // useInterval(() => {
  //   const connection = navigator['connection']

  //   setState({
  //     rtt: connection.rtt,
  //     type: connection.type,
  //     effectiveType: connection.effectiveType,
  //   })
  // }, 1000)

  return { ...state }
}

const NetworkInfo = () => {
  const connection = useConnection()

  const NetworkIcon: FC = () => {
    switch (connection.type) {
      case 'bluetooth':
        switch (connection.effectiveType) {
          case 'slow-2g':
          case '2g':
          case '3g':
          case '4g':
            return <MdBluetoothConnected />
          default:
            return <MdBluetoothDisabled />
        }
      case 'cellular':
        switch (connection.effectiveType) {
          case 'slow-2g':
          case '2g':
          case '3g':
          case '4g':
            return <MdSignalCellular4Bar />
          default:
            return <MdSignalCellularOff />
        }
      case 'ethernet':
        return <MdSettingsEthernet />
      case 'wifi':
      default:
        switch (connection.effectiveType) {
          case 'slow-2g':
            return <MdSignalWifi1Bar />
          case '2g':
            return <MdSignalWifi2Bar />
          case '3g':
            return <MdSignalWifi3Bar />
          case '4g':
            return <MdSignalWifi4Bar />
          default:
            return <MdSignalWifiOff />
        }
    }
  }

  return (
    <div>
      {connection.rtt}ms &nbsp;
      <NetworkIcon />
    </div>
  )
}

export const NetworkDisplay = () => {
  if (navigator) return <NetworkInfo />
  return null
}
