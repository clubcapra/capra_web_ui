import { Twist, Vector3 } from './types'
import CustomGamepad from '@/utils/gamepad/CustomGamepad'
import { Stick, GamepadBtn } from '../gamepad/mappings/types'

export const mapGamepadToTwist = (gamepad: CustomGamepad): Twist => {
  const leftStick = gamepad.getStick(Stick.Left)
  const rt = gamepad.getButtonValue(GamepadBtn.RT)
  const lt = gamepad.getButtonValue(GamepadBtn.LT)

  const x =
    leftStick.horizontal > 0.15 ? -1 : leftStick.horizontal < -0.15 ? 1 : 0
  const y = leftStick.vertical > 0.15 ? 1 : leftStick.vertical < -0.15 ? -1 : 0

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
