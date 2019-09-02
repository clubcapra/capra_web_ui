import React, { useState, FC } from 'react'
import {
  MdBluetoothConnected,
  MdBluetoothDisabled,
  MdSignalCellular0Bar,
  MdSignalCellularOff,
  MdSignalWifi1Bar,
  MdSignalWifi2Bar,
  MdSignalWifi3Bar,
  MdSignalWifi4Bar,
  MdSignalWifiOff,
  MdSettingsEthernet,
} from 'react-icons/md'
import { useInterval } from 'utils/hooks/useInterval'

const NetworkInfo = () => {
  // @ts-ignore
  const { connection } = navigator

  const [state, setState] = useState({
    networkRTT: connection.rtt,
    connectionType: connection.type,
    connectionEffectiveType: connection.effectiveType,
  })

  useInterval(() => {
    //@ts-ignore
    const { connection } = navigator

    setState({
      networkRTT: connection.rtt,
      connectionType: connection.type,
      connectionEffectiveType: connection.effectiveType,
    })
  }, 1000)

  const NetworkIcon: FC = () => {
    switch (state.connectionType) {
      case 'bluetooth':
        switch (state.connectionEffectiveType) {
          case 'slow-2g':
          case '2g':
          case '3g':
          case '4g':
            return <MdBluetoothConnected />
          default:
            return <MdBluetoothDisabled />
        }
      case 'cellular':
        switch (state.connectionEffectiveType) {
          case 'slow-2g':
          case '2g':
          case '3g':
          case '4g':
            return <MdSignalCellular0Bar />
          default:
            return <MdSignalCellularOff />
        }
      case 'ethernet':
        return <MdSettingsEthernet />
      case 'wifi':
      default:
        switch (state.connectionEffectiveType) {
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
      {state.networkRTT}ms &nbsp;
      <NetworkIcon />
    </div>
  )
}

export default NetworkInfo
