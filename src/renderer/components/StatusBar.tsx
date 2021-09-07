import { styled } from '@/renderer/globalStyles/styled'
import { controlService } from '@/renderer/state/control'
import { flipperService } from '@/renderer/state/flipper'
import { rosService } from '@/renderer/state/ros'
import { selectFullAddress } from '@/renderer/store/modules/ros'
import { useSelector } from '@/renderer/utils/hooks/typedUseSelector'
import { useInterval } from '@/renderer/utils/hooks/useInterval'
import { useActor } from '@xstate/react'
import { format } from 'date-fns'
import React, { FC, useState } from 'react'
import { BiWifi, BiWifi0, BiWifi1, BiWifi2, BiWifiOff } from 'react-icons/bi'

export const StatusBar: FC = () => (
  <StyledStatusBarWrapper>
    <LeftStatusBar>
      <RosConnectionStatus />
    </LeftStatusBar>
    <RightStatusBar>
      <ControlStatus />
      <FlipperMode />
      <NetworkInfo />
      <TimeDisplay />
    </RightStatusBar>
  </StyledStatusBarWrapper>
)

const RosConnectionStatus: FC = () => {
  const [state, send] = useActor(rosService)
  const fullAddress = useSelector(selectFullAddress)
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
  const [flipper] = useActor(flipperService)
  const [control] = useActor(controlService)
  if (control.matches('flipper')) {
    return (
      <div>
        {flipper.matches('front') && 'FRONT'}
        {flipper.matches('none') && 'NONE'}
        {flipper.matches('back') && 'BACK'}
      </div>
    )
  } else {
    return <div />
  }
}

const NetworkInfo = () => {
  const [effectiveType, setEffectiveType] = useState('4g')

  useInterval(() => {
    // eslint-disable-next-line @typescript-eslint/ban-ts-comment
    // @ts-ignore
    setEffectiveType(navigator.connection.effectiveType)
  }, 1000)

  if (navigator) {
    const NetworkIcon = () => {
      switch (effectiveType) {
        case 'slow-2g':
          return <BiWifi0 />
        case '2g':
          return <BiWifi1 />
        case '3g':
          return <BiWifi2 />
        case '4g':
          return <BiWifi />
        default:
          return <BiWifiOff />
      }
    }

    return (
      <div>
        <NetworkIcon />
      </div>
    )
  }
  return null
}

const timeFormat = (date: Date) => format(date, 'HH:mm:ss')
const TimeDisplay: FC = () => {
  const [time, setTime] = useState(timeFormat(new Date()))

  useInterval(() => {
    setTime(timeFormat(new Date()))
  }, 1000)

  return <div>{time}</div>
}

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
