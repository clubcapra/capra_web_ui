import CustomGamepad from './CustomGamepad'
import { Stick, GamepadBtn } from './mappings/types'
import { Twist, Vector3 } from '@/utils/math/types'

let joySeqId = 0

export const mapGamepadToJoy = (gamepad: Gamepad) => {
  const d = new Date()
  const seconds = Math.round(d.getTime() / 1000)
  const cgamepad = new CustomGamepad(gamepad)

  const axes = gamepad.axes.map(x => (x < 0.09 && x > -0.09 ? 0.0 : x))

  //Add Trigger axis at the right place in the axes array
  // axs.splice(2,0,cgamepad.getButtonValue(GamepadBtn.LT))
  // axs.splice(5,0,cgamepad.getButtonValue(GamepadBtn.RT))

  const buttons = gamepad.buttons.map(x => Math.floor(x.value))

  const joyMsg = {
    header: {
      seq: joySeqId++,
      stamp: {
        sec: seconds,
        nsecs: 0,
      },
      frame_id: '',
    },
    axes: axes,
    buttons: buttons,
  }

  return joyMsg
}

export const mapGamepadToTwist = (gamepad: CustomGamepad): Twist => {
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
