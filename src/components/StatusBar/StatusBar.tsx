import React, { FC } from 'react'
import {
  StyledStatusBarWrapper,
  RightStatusBar,
  LeftStatusBar,
} from './StatusBar.styles'
import { useSelector } from 'react-redux'
import { fullRobotIpAddress } from 'store/modules/ros/reducer'
import { GlobalState } from 'store/rootReducer'
import { TimeDisplay } from 'components/StatusBar/TimeDisplay'

export const StatusBar: FC = () => {
  const robotIpAddress = useSelector(fullRobotIpAddress)
  const connected = useSelector((state: GlobalState) => state.ros.connected)

  return (
    <StyledStatusBarWrapper>
      <LeftStatusBar>
        {connected ? `Connected to: ${robotIpAddress}` : 'Disconnected'}
      </LeftStatusBar>
      <RightStatusBar>
        <TimeDisplay />
      </RightStatusBar>
    </StyledStatusBarWrapper>
  )
}
