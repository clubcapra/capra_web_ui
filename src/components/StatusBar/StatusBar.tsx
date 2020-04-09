import React, { FC } from 'react'
import {
  StyledStatusBarWrapper,
  RightStatusBar,
  LeftStatusBar,
} from './StatusBar.styles'
import { TimeDisplay } from 'components/StatusBar/TimeDisplay'
import { useSelector } from 'utils/hooks/typedUseSelector'
import { NetworkDisplay } from './NetworkInfo'
import { FaTruckMonster, FaHandPaper } from 'react-icons/fa'
import { useService } from '@xstate/react'
import { rosService, fullAddressSelector } from 'state/ros'

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
  const isArmControlled = useSelector((state) => state.gamepad.isArmControlled)
  return <div>{isArmControlled ? <FaHandPaper /> : <FaTruckMonster />}</div>
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
