import React, { FC } from 'react'
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
import { useConnection } from 'utils/hooks/useConnection'

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
  //@ts-ignore
  if (navigator && navigator.connection) return <NetworkInfo />
  return null
}
