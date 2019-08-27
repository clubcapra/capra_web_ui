import React, { FC } from 'react'
import {
  StyledStatusBarWrapper,
  RightStatusBar,
  LeftStatusBar,
} from './StatusBar.styles'
import { fullIpAddress } from 'store/modules/ros/reducer'
import { TimeDisplay } from 'components/StatusBar/TimeDisplay'
import { useSelector } from 'utils/hooks/typedUseSelector'

const RosConnectionStatus: FC = () => {
  const robotIpAddress = useSelector(fullIpAddress)
  const connected = useSelector(state => state.ros.connected)
  const tryingToConnect = useSelector(state => state.ros.tryingToConnect)

  return (
    <div>
      {connected
        ? `Connected to: ${robotIpAddress}`
        : tryingToConnect
        ? `Trying to connect to ${robotIpAddress}`
        : 'Disconnected'}
    </div>
  )
}

export const StatusBar: FC = () => {
  return (
    <StyledStatusBarWrapper>
      <LeftStatusBar>
        <RosConnectionStatus />
      </LeftStatusBar>
      <RightStatusBar>
        <TimeDisplay />
      </RightStatusBar>
    </StyledStatusBarWrapper>
  )
}
