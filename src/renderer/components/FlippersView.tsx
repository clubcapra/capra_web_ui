import { styled } from '@/renderer/globalStyles/styled'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import { useRosSubscribe } from '@/renderer/hooks/useRosSubscribe'
import React, { FC, useCallback, useEffect, useState } from 'react'
import { selectReverse } from '@/renderer/store/modules/input'
import { useSelector } from '@/renderer/hooks/typedUseSelector'
import ROSLIB from 'roslib'
import { rosClient } from '../utils/ros/rosClient'

interface Props {
  flipper: IFlipperData
  name: string
}

const flipperUpperLimitParam = new ROSLIB.Param({
  name: 'markhor/flippers/markhor_flippers_node/front_left_drive_upper_limit',
  ros: rosClient.ros,
})

const flipperLowerLimitParam = new ROSLIB.Param({
  name: 'markhor/flippers/markhor_flippers_node/front_left_drive_lower_limit',
  ros: rosClient.ros,
})

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
      <StyledTLArea flipper={flippers.flipperRR} name="Front right" />
      <StyledTRArea flipper={flippers.flipperRL} name="Front left" />
      <StyledBLArea flipper={flippers.flipperFL} name="Rear left" />
      <StyledBRArea flipper={flippers.flipperFR} name="Rear right" />
    </>
  )
}

const FlipperArea: FC<Props> = ({ flipper, name }) => {
  const [position, setPosition] = useState<string>('0.00')
  const [motorCurrentColor, setMotorCurrentColor] = useState<string>('')
  const [motorCurrentValue, setMotorCurrentValue] = useState<string>('0')
  const [flipperUpperLimit, setFlipperUpperLimit] = useState<number>(1)
  const [flipperLowerLimit, setFlipperLowerLimit] = useState<number>(1)

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
  }, [position, motorCurrentValue, motorCurrentColor])

  // Load flipper limits from ROS
  useEffect(() => {
    flipperLowerLimitParam.get((value) => {
      if (value) {
        setFlipperLowerLimit(value as number)
      }
    })
    flipperUpperLimitParam.get((value) => {
      if (value) {
        setFlipperUpperLimit(value as number)
      }
    })
  }, [])

  useRosSubscribe(
    flipper.topicPosition,
    useCallback(
      (message) => {
        setPosition(
          (
            (Number(message.data) /
              (Number(message.data) < 0
                ? flipperUpperLimit
                : flipperLowerLimit)) *
            -100
          )
            .toFixed(2)
            .replace(/\B(?<!\.\d*)(?=(\d{3})+(?!\d))/g, ' ')
        )
      },
      [flipperLowerLimit, flipperUpperLimit]
    )
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
      <StyledName>{name}</StyledName>
      <StyledPostion>{position} %</StyledPostion>
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
  align-items: flex-end;
  justify-items: flex-end;
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
    'mcc mcv'
    'x t';
  padding: 2px;
`
const StyledName = styled.p`
  grid-area: n;
  margin-right: 5px;
`

const StyledPostion = styled.p`
  grid-area: p;
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
