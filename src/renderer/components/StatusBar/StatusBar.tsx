import { useActor } from '@xstate/react'
import { TimeDisplay } from '@/renderer/components/StatusBar/TimeDisplay'
import React, { FC } from 'react'
import { controlService } from '@/renderer/state/control'
import { fullAddressSelector, rosService } from '@/renderer/state/ros'
import { NetworkDisplay } from './NetworkInfo'
import { styled } from '@/renderer/globalStyles/styled'
import { flipperService } from '@/renderer/state/flipper'

const RosConnectionStatus: FC = () => {
  const [state, send] = useActor(rosService)
  const fullAddress = fullAddressSelector(state.context)
  const connect = () => {
    send('CONNECT')
  }

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
  const [state] = useActor(controlService)
  return (
    <div>
      {state.matches('arm') && 'ARM'}
      {state.matches('flipper') && 'FLIPPER'}
      {state.matches('nothing') && 'NOTHING'}
    </div>
  )
}

const FlipperMode = () => {
  const [state] = useActor(flipperService)
  return (
    <div>
      {state.matches('front') && 'FRONT'}
      {state.matches('none') && 'NONE'}
      {state.matches('back') && 'BACK'}
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
      <FlipperMode />
      <NetworkDisplay />
      <TimeDisplay />
    </RightStatusBar>
  </StyledStatusBarWrapper>
)

const StyledStatusBarWrapper = styled.div`
  display: grid;
  grid-template: 'l r';
  grid-template-columns: auto 1fr;
  height: 100%;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  color: ${({ theme }) => theme.colors.fontLight};
  box-shadow: 0 -2px 2px rgba(0, 0, 0, 0.25);
  font-size: 14px;
`

const padding = 6

const LeftStatusBar = styled.div`
  grid-area: l;
  display: flex;
  align-items: center;
  padding: 0 ${padding}px;
`

const RightStatusBar = styled.div`
  grid-area: r;
  display: flex;
  justify-items: center;
  justify-content: flex-end;

  > * {
    padding: 0 ${padding}px;
  }
`
