import { styled } from '@/renderer/globalStyles/styled'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import { useRosSubscribe } from '@/renderer/hooks/useRosSubscribe'
import React, { FC, useCallback, useEffect, useState } from 'react'
import { selectReverse } from '@/renderer/store/modules/input'
import { useSelector } from '@/renderer/hooks/typedUseSelector'

interface Props {
  flipper: IFlipperData
  target: string
}

export const FlippersBar: FC = () => {
  const isReverse = useSelector(selectReverse)
  return (
    <StyledFlippersBarWrapper>
      {isReverse ? <FlippersReverseView /> : <FlippersForwardView />}
    </StyledFlippersBarWrapper>
  )
}

const FlippersForwardView: FC = () => {
  return (
    <>
      <FlipperArea flipper={flippers.flipperRL} target="Rear left" />
      <FlipperArea flipper={flippers.flipperRR} target="Rear right" />
      <FlipperArea flipper={flippers.flipperFR} target="Front right" />
      <FlipperArea flipper={flippers.flipperFL} target="Front left" />
    </>
  )
}

const FlippersReverseView: FC = () => {
  return (
    <>
      <FlipperArea flipper={flippers.flipperRR} target="Rear left" />
      <FlipperArea flipper={flippers.flipperFR} target="Rear right" />
      <FlipperArea flipper={flippers.flipperRL} target="Front right" />
      <FlipperArea flipper={flippers.flipperFL} target="Front left" />
    </>
  )
}

const FlipperArea: FC<Props> = ({ flipper, target }) => {
  const [position, setPosition] = useState<string>('')
  const [motorCurrentColor, setMotorCurrentColor] = useState<string>('')
  const [motorCurrentValue, setMotorCurrentValue] = useState<string>('')

  useEffect(() => {
    if (!position) {
      setPosition('')
    }
    if (!motorCurrentValue) {
      setMotorCurrentValue('')
    }
    if (!motorCurrentColor) {
      setMotorCurrentColor('')
    }
  }, [position, motorCurrentValue, motorCurrentColor])

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

  return (
    <StyledFlipperArea>
      <StyledTarget>{target}:</StyledTarget>
      <StyledPostion>{position}</StyledPostion>
      <StyledMotorCurrentColor style={{ backgroundColor: motorCurrentColor }} />
      <StyledMotorCurrentValue>{motorCurrentValue}</StyledMotorCurrentValue>
    </StyledFlipperArea>
  )
}

const StyledFlippersBarWrapper = styled.div`
  display: grid;
  grid-template-columns: 1fr 1fr 1fr 1fr;
  align-items: center;
  justify-items: center;
  height: 100%;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  color: ${({ theme }) => theme.colors.fontLight};
  box-shadow: 0 -2px 2px rgba(0, 0, 0, 0.25);
  font-size: 14px;
`

const StyledFlipperArea = styled.div`
  height: 100%;
  padding: 5px;
  display: grid;
  grid-template-areas:
    't p'
    'mcc mcv';
`

const StyledTarget = styled.p`
  grid-area: t;
  margin-right: 5px;
`

const StyledPostion = styled.p`
  grid-area: p;
`

const StyledMotorCurrentColor = styled.p`
  grid-area: mcc;
  border-radius: 5px;
  margin: 5px;
`

const StyledMotorCurrentValue = styled.p`
  grid-area: mcv;
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
  },
}
