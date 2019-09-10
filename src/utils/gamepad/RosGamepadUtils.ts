import { Stick, GamepadData } from 'utils/gamepad/@types'
import { Vector3 } from 'utils/math/types'
import { TopicOptions } from '@club_capra/roslib-ts-client'
import { IJoyMsg, ITwistMsg } from 'utils/ros/rosMsgs.types'
import { GamepadBtn } from './@types'
import { getStick, getButtonValue } from 'utils/gamepad/GamepadUtils'

export const cmdVelTopic: TopicOptions = {
  name: '/cmd_vel',
  messageType: 'geometry_msgs/Twist',
}

export const joyTopic: TopicOptions = {
  name: '/joy',
  messageType: 'sensor_msgs/Joy',
}

export const spaceMouseTopic: TopicOptions = {
  name: '/spacenav/twist',
  messageType: 'geometry_msgs/Twist',
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

export const mapGamepadToTwist = (gamepad: GamepadData): ITwistMsg => {
  const { horizontal, vertical } = getStick(Stick.Left)(gamepad)
  const rt = getButtonValue(GamepadBtn.RT)(gamepad)
  const lt = getButtonValue(GamepadBtn.LT)(gamepad)

  const x = horizontal > 0.15 ? -1 : horizontal < -0.15 ? 1 : 0
  const y = vertical > 0.15 ? 1 : vertical < -0.15 ? -1 : 0

  if (lt > 0.1) {
    return { linear: Vector3.zero(), angular: Vector3.zero() }
  }

  return {
    linear: {
      x: y * rt,
      y: 0,
      z: 0,
    },
    angular: {
      x: 0,
      y: 0,
      z: x * rt,
    },
  }
}

export const mapSpaceMouseToTwist = (spacemouse: Gamepad): ITwistMsg => ({
  linear: {
    x: spacemouse.axes[0],
    y: spacemouse.axes[1],
    z: spacemouse.axes[2],
  },
  angular: {
    x: spacemouse.axes[3],
    y: spacemouse.axes[4],
    z: spacemouse.axes[5],
  },
})
