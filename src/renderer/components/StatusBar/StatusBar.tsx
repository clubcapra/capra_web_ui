import { useService } from '@xstate/react'
import { TimeDisplay } from '@/renderer/components/StatusBar/TimeDisplay'
import React, { FC } from 'react'
import { controlService } from '@/renderer/state/control'
import { fullAddressSelector, rosService } from '@/renderer/state/ros'
import { NetworkDisplay } from './NetworkInfo'
import {
  LeftStatusBar,
  RightStatusBar,
  StyledStatusBarWrapper,
} from './StatusBar.styles'

const RosConnectionStatus: FC = () => {
  const [state, send] = useService(rosService)
  const fullAddress = fullAddressSelector(state.context)
  const connect = () => send('CONNECT')

  return (
    <>
      <div onClick={connect}>
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
    <div>
      {state.matches('arm') && 'ARM'}
      {state.matches('flipper') && 'FLIPPER'}
      {state.matches('nothing') && 'NOTHING'}
    </div>
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
