import { styled } from '@/renderer/globalStyles/styled'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import { useRosSubscribe } from '@/renderer/hooks/useRosSubscribe'
import React, { FC, useCallback, useEffect, useState } from 'react'
import { selectReverse } from '@/renderer/store/modules/input'
import { useSelector } from '@/renderer/hooks/typedUseSelector'

interface Props {
  flipper: IFlipperData
  name: string
}

export const FlippersView: FC = () => {
  const isReverse = useSelector(selectReverse)
  return (
    <StyledFlippersView>
      {isReverse ? <FlippersReverseView /> : <FlippersForwardView />}
    </StyledFlippersView>
  )
}

const FlippersForwardView: FC = () => {
  return (
    <>
      <StyledTLArea flipper={flippers.flipperFR} name="Front right" />
      <StyledTRArea flipper={flippers.flipperFL} name="Front left" />
      <StyledBLArea flipper={flippers.flipperRL} name="Rear left" />
      <StyledBRArea flipper={flippers.flipperRR} name="Rear right" />
    </>
  )
}

const FlippersReverseView: FC = () => {
  return (
    <>
      <StyledTLArea flipper={flippers.flipperFR} name="Front right" />
      <StyledTRArea flipper={flippers.flipperFL} name="Front left" />
      <StyledBLArea flipper={flippers.flipperRL} name="Rear left" />
      <StyledBRArea flipper={flippers.flipperFR} name="Rear right" />
    </>
  )
}

const FlipperArea: FC<Props> = ({ flipper, name }) => {
  const [position, setPosition] = useState<string>('0.00')
  const [motorCurrentColor, setMotorCurrentColor] = useState<string>('')
  const [motorCurrentValue, setMotorCurrentValue] = useState<string>('0')
  const [MotorBusVoltage, setMotorBusVoltage] = useState<string>('0')

  useEffect(() => {
    if (!position) {
      setPosition('0')
    }
    if (!motorCurrentValue) {
      setMotorCurrentValue('0')
      setMotorCurrentColor('')
    }
    if (!motorCurrentColor) {
      setMotorCurrentColor('')
    }
    if (!MotorBusVoltage) {
      setMotorBusVoltage('0')
    }
  }, [position, motorCurrentValue, motorCurrentColor, MotorBusVoltage])

  useRosSubscribe(
    flipper.topicPosition,
    useCallback((message) => {
      setPosition(Number(message.data).toFixed(2))
    }, [])
  )

  useRosSubscribe(
    flipper.topicMotorCurrent,
    useCallback((message) => {
      const motorCurrent = Number(message.data)
      const maxCurrent = 30
      const red = '#ff0000'
      const color =
        red +
        Math.floor(
          Math.min(Math.abs(motorCurrent / maxCurrent) * 255, 255)
        ).toString(16)
      setMotorCurrentColor(color)
      setMotorCurrentValue(motorCurrent.toFixed(2))
    }, [])
  )

  useRosSubscribe(
    flipper.topicMotorBusVoltage,
    useCallback((message) => {
      setMotorBusVoltage(Number(message.data).toFixed(2))
    }, [])
  )

  return (
    <StyledFlipperArea>
      <StyledName>{name}</StyledName>
      <StyledPostion>{position}</StyledPostion>
      {/* <StyledMotorBusVoltage>{MotorBusVoltage} V</StyledMotorBusVoltage> */}
      <StyledMotorCurrentColor style={{ backgroundColor: motorCurrentColor }} />
      <StyledMotorCurrentValue>{motorCurrentValue} A</StyledMotorCurrentValue>
    </StyledFlipperArea>
  )
}

const StyledFlippersView = styled.div`
  display: grid;
  grid-template-areas:
    'tl tr'
    'bl br';
  grid-gap: 1px;
  align-items: center;
  justify-items: center;
  height: 100%;
  padding: 2px;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  color: ${({ theme }) => theme.colors.fontLight};
  box-shadow: 0 -2px 2px rgba(0, 0, 0, 0.25);
  font-size: 14px;
`

const StyledTLArea = styled(FlipperArea)`
  grid-area: tl;
`

const StyledTRArea = styled(FlipperArea)`
  grid-area: tl;
`

const StyledBLArea = styled(FlipperArea)`
  grid-area: tl;
`

const StyledBRArea = styled(FlipperArea)`
  grid-area: tl;
`

const StyledFlipperArea = styled.div`
  height: 100%;
  width: 100%;
  display: grid;
  grid-template-areas:
    'n p'
    // '1fr t'
    'mcc mcv';
`
const StyledName = styled.p`
  grid-area: n;
`

const StyledPostion = styled.p`
  grid-area: p;
`

const StyledMotorBusVoltage = styled.p`
  grid-area: t;
`

const StyledMotorCurrentValue = styled.p`
  grid-area: mcv;
`

const StyledMotorCurrentColor = styled.p`
  grid-area: mcc;
  border-radius: 5px;
  margin: 5px;
`

export interface IFlippers {
  id: string
  flipperFL: IFlipperData
  flipperFR: IFlipperData
  flipperRL: IFlipperData
  flipperRR: IFlipperData
}

export interface IFlipperData {
  topicPosition: TopicOptions<string>
  topicMotorCurrent: TopicOptions<string>
  topicMotorBusVoltage: TopicOptions<string>
}

const flippers: IFlippers = {
  id: 'flippers',
  flipperFL: {
    topicPosition: {
      name: '/markhor/flippers/flipper_fl_position_target',
      messageType: 'std_msgs/Float64',
    },
    topicMotorCurrent: {
      name: '/markhor/flippers/flipper_fl_motor_current',
      messageType: 'std_msgs/Float64',
    },
    topicMotorBusVoltage: {
      name: '/markhor/flippers/flipper_fl_bus_voltage',
      messageType: 'std_msgs/Float64',
    },
  },
  flipperFR: {
    topicPosition: {
      name: '/markhor/flippers/flipper_fr_position_target',
      messageType: 'std_msgs/Float64',
    },
    topicMotorCurrent: {
      name: '/markhor/flippers/flipper_fr_motor_current',
      messageType: 'std_msgs/Float64',
    },
    topicMotorBusVoltage: {
      name: '/markhor/flippers/flipper_fr_bus_voltage',
      messageType: 'std_msgs/Float64',
    },
  },
  flipperRL: {
    topicPosition: {
      name: '/markhor/flippers/flipper_rl_position_target',
      messageType: 'std_msgs/Float64',
    },
    topicMotorCurrent: {
      name: '/markhor/flippers/flipper_rl_motor_current',
      messageType: 'std_msgs/Float64',
    },
    topicMotorBusVoltage: {
      name: '/markhor/flippers/flipper_rl_bus_voltage',
      messageType: 'std_msgs/Float64',
    },
  },
  flipperRR: {
    topicPosition: {
      name: '/markhor/flippers/flipper_rr_position_target',
      messageType: 'std_msgs/Float64',
    },
    topicMotorCurrent: {
      name: '/markhor/flippers/flipper_rr_motor_current',
      messageType: 'std_msgs/Float64',
    },
    topicMotorBusVoltage: {
      name: '/markhor/flippers/flipper_rr_bus_voltage',
      messageType: 'std_msgs/Float64',
    },
  },
}
