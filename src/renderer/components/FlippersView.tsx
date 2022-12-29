import { styled } from '@/renderer/globalStyles/styled'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import {
  useRosSubscribe,
  useRosSubscribeNoData,
} from '@/renderer/hooks/useRosSubscribe'
import React, { FC, useCallback, useEffect, useState } from 'react'
import { selectReverse } from '@/renderer/store/modules/input'
import { useSelector } from '@/renderer/hooks/typedUseSelector'
import ROSLIB from 'roslib'
import { rosClient } from '../utils/ros/rosClient'
import { selectRobotName } from '../store/modules/ros'

interface Props {
  flipper: IFlipperData
  name: string
  flipperPosition: number
}

interface ViewProps {
  flipperPositions: number[]
}

interface PositionsMsg {
  positions: number[]
}

export const FlippersView: FC = () => {
  const isReverse = useSelector(selectReverse)
  const [flipperPositions, setFlipperPositions] = useState<number[]>([])
  const robotName = useSelector(selectRobotName)

  const flipperPositionTopic: TopicOptions = {
    name: { robotName } + '/flippers/flipper_positions',
    messageType: '/' + { robotName } + '_flippers/FlipperPositions',
  }

  useRosSubscribeNoData<PositionsMsg>(
    flipperPositionTopic,
    useCallback((message) => {
      if (message.positions[0] !== flipperPositions[0]) {
        setFlipperPositions(message.positions)
      }
    }, [])
  )
  return (
    <StyledFlippersView>
      {isReverse ? (
        <FlippersReverseView flipperPositions={flipperPositions} />
      ) : (
        <FlippersForwardView flipperPositions={flipperPositions} />
      )}
    </StyledFlippersView>
  )
}

const FlippersForwardView: FC<ViewProps> = ({ flipperPositions }) => {
  return (
    <>
      <StyledTRArea
        flipper={flippers.flipperFL}
        name="Front left"
        flipperPosition={flipperPositions[0]}
      />
      <StyledTLArea
        flipper={flippers.flipperFR}
        name="Front right"
        flipperPosition={flipperPositions[1]}
      />
      <StyledBLArea
        flipper={flippers.flipperRL}
        name="Rear left"
        flipperPosition={flipperPositions[2]}
      />
      <StyledBRArea
        flipper={flippers.flipperRR}
        name="Rear right"
        flipperPosition={flipperPositions[3]}
      />
    </>
  )
}

const FlippersReverseView: FC<ViewProps> = ({ flipperPositions }) => {
  return (
    <>
      <StyledTRArea
        flipper={flippers.flipperRL}
        name="Front left"
        flipperPosition={flipperPositions[0]}
      />
      <StyledTLArea
        flipper={flippers.flipperRR}
        name="Front right"
        flipperPosition={flipperPositions[1]}
      />
      <StyledBLArea
        flipper={flippers.flipperFL}
        name="Rear left"
        flipperPosition={flipperPositions[2]}
      />
      <StyledBRArea
        flipper={flippers.flipperFR}
        name="Rear right"
        flipperPosition={flipperPositions[3]}
      />
    </>
  )
}

const FlipperArea: FC<Props> = ({ flipper, name, flipperPosition }) => {
  const [position, setPosition] = useState<string>('0.00')
  const [motorCurrentColor, setMotorCurrentColor] = useState<string>('')
  const [motorCurrentValue, setMotorCurrentValue] = useState<string>('0')
  const [flipperUpperLimit, setFlipperUpperLimit] = useState<number>(0)
  const [flipperLowerLimit, setFlipperLowerLimit] = useState<number>(0)
  const robotName = useSelector(selectRobotName)

  const flipperUpperLimitParam = new ROSLIB.Param({
    name: { robotName } + '/flippers/' + { robotName } + '_flippers_node/front_left_drive_upper_limit',
    ros: rosClient.ros,
  })

  const flipperLowerLimitParam = new ROSLIB.Param({
    name: { robotName } + '/flippers/' + { robotName } + '_flippers_node/front_left_drive_lower_limit',
    ros: rosClient.ros,
  })

  useEffect(() => {
    if (!flipperPosition) {
      setPosition('0')
    } else if (flipperLowerLimit && flipperUpperLimit) {
      setPosition(
        (
          (flipperPosition /
            (flipperPosition < 0 ? flipperUpperLimit : flipperLowerLimit)) *
          -100
        ).toFixed(2)
      )
    }
    if (!motorCurrentValue) {
      setMotorCurrentValue('0')
      setMotorCurrentColor('')
    }
    if (!motorCurrentColor) {
      setMotorCurrentColor('')
    }
  }, [
    motorCurrentValue,
    motorCurrentColor,
    flipperPosition,
    flipperUpperLimit,
    flipperLowerLimit,
  ])

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
    'mcc mcv';
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
  topicMotorCurrent: TopicOptions<string>
}

const flippers: IFlippers = {
  id: 'flippers',
  flipperFL: {
    topicMotorCurrent: {
      name: '/markhor/flippers/flipper_fl_motor_current',
      messageType: 'std_msgs/Float64',
    },
  },
  flipperFR: {
    topicMotorCurrent: {
      name: '/markhor/flippers/flipper_fr_motor_current',
      messageType: 'std_msgs/Float64',
    },
  },
  flipperRL: {
    topicMotorCurrent: {
      name: '/markhor/flippers/flipper_rl_motor_current',
      messageType: 'std_msgs/Float64',
    },
  },
  flipperRR: {
    topicMotorCurrent: {
      name: '/markhor/flippers/flipper_rr_motor_current',
      messageType: 'std_msgs/Float64',
    },
  },
}
