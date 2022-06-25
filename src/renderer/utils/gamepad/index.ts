import { selectReverse } from '@/renderer/store/modules/input'
import { ITwistMsg } from '@/renderer/utils/ros/rosMsgs.types'
import { Vector3 } from '@/renderer/utils/math/types'
import { store } from '@/renderer/store/store'

export const deadzone = (value: number): number => {
  const deadzone = 0.1
  return value > deadzone || value < -deadzone ? value : 0
}

export const mapToTwist = (
  horizontal: number,
  vertical: number,
  rt: number,
  lt: number,
  dpadLeft: number,
  dpadRight: number,
  turboEnabled: boolean
): ITwistMsg => {
  const isReverse = selectReverse(store.getState())
  let x = horizontal
  x = isReverse ? -x : x
  let y = vertical
  y = isReverse ? -y : y

  if (lt > 0.1) {
    // brake!
    return { linear: Vector3.zero(), angular: Vector3.zero() }
  }

  const linearSensitivity = turboEnabled ? 1 : 0.2
  const angularSensitivity = turboEnabled ? 1 : 0.7

  let linearX = deadzone(y * rt) * linearSensitivity
  let angularZ = deadzone(x * rt) * angularSensitivity

  if (dpadLeft > 0.0) {
    linearX = 0
    angularZ = isReverse ? -rt : rt
  }
  if (dpadRight > 0.0) {
    linearX = 0
    angularZ = isReverse ? rt : -rt
  }

  return {
    linear: new Vector3(linearX, 0, 0),
    angular: new Vector3(0, 0, angularZ),
  }
}
