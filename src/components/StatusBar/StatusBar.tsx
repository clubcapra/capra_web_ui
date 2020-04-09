import React, { FC } from 'react'
import {
  StyledStatusBarWrapper,
  RightStatusBar,
  LeftStatusBar,
} from './StatusBar.styles'
import { TimeDisplay } from 'components/StatusBar/TimeDisplay'
import { NetworkDisplay } from './NetworkInfo'
import { FaTruckMonster, FaHandPaper } from 'react-icons/fa'
import { useService } from '@xstate/react'
import { rosService, fullAddressSelector } from 'state/ros'
import { controlService } from 'state/control'

const RosConnectionStatus: FC = () => {
  const [state] = useService(rosService)
  const fullAddress = fullAddressSelector(state.context)

  return (
    <>
      <div>
        {state.matches('connected') && `Connected to ${fullAddress}`}
        {state.matches('connecting') && `Trying to connect to ${fullAddress}`}
        {state.matches('disconnected') && `Disconnected`}
      </div>
    </>
  )
}

const ControlStatus = () => {
  const [state] = useService(controlService)
  return (
    <div>{state.matches('arm') ? <FaHandPaper /> : <FaTruckMonster />}</div>
  )
}

export const StatusBar: FC = () => (
  <StyledStatusBarWrapper>
    <LeftStatusBar>
      <RosConnectionStatus />
    </LeftStatusBar>
    <RightStatusBar>
      <ControlStatus />
      <NetworkDisplay />
      <TimeDisplay />
    </RightStatusBar>
  </StyledStatusBarWrapper>
)
