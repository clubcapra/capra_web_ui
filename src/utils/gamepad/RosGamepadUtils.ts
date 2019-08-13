import { Vector3 } from 'utils/math/types'
import CustomGamepad from './CustomGamepad'
import { GamepadBtn, Stick } from './mappings/types'
import { TopicOptions } from '@club_capra/roslib-ts-client'
import { IJoyMsg, ITwistMsg, Twist } from 'utils/ros/rosMsgs.types'

export const cmdVelTopic: TopicOptions = {
  name: '/cmd_vel',
  messageType: 'geometry_msgs/Twist',
}

export const joyTopic: TopicOptions = {
  name: '/joy',
  messageType: 'sensor_msgs/Joy',
}

let joySeqId = 0

export const mapGamepadToJoy = (gamepad: Gamepad): IJoyMsg => {
  const d = new Date()
  const seconds = Math.round(d.getTime() / 1000)

  const axes = gamepad.axes.map(x => (x < 0.09 && x > -0.09 ? 0.0 : x))

  const buttons = gamepad.buttons.map(x => Math.floor(x.value))

  return {
    header: {
      seq: joySeqId++,
      stamp: {
        sec: seconds,
        nsecs: 0,
      },
      // eslint-disable-next-line @typescript-eslint/camelcase
      frame_id: '',
    },
    axes: axes,
    buttons: buttons,
  }
}

export const mapGamepadToTwist = (gamepad: CustomGamepad): ITwistMsg => {
  const { horizontal, vertical } = gamepad.getStick(Stick.Left)
  const rt = gamepad.getButtonValue(GamepadBtn.RT)
  const lt = gamepad.getButtonValue(GamepadBtn.LT)

  const x = horizontal > 0.15 ? -1 : horizontal < -0.15 ? 1 : 0
  const y = vertical > 0.15 ? 1 : vertical < -0.15 ? -1 : 0

  if (lt > 0.1) {
    return new Twist(Vector3.zero(), Vector3.zero())
  }

  const linear = {
    x: y * rt,
    y: 0,
    z: 0,
  }

  const angular = {
    x: 0,
    y: 0,
    z: x * rt,
  }

  return new Twist(linear, angular)
}
