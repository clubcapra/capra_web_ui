import { InputSystem } from '@/renderer/inputSystem/InputSystem'
import {
  buttons as buttonMappings,
  sticks,
} from '@/renderer/inputSystem/mappings'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import { Action } from '@/renderer/inputSystem/@types'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import { ITwistMsg, IJoyMsg } from '@/renderer/utils/ros/rosMsgs.types'
import { Vector3 } from '@/renderer/utils/math/types'
import { controlService } from '@/renderer/state/control'
import { feedSlice } from '@/renderer/store/modules/feed/reducer'
import { store } from '@/renderer/store/store'
import { flipperService } from '@/renderer/state/flipper'

const cmdVelTopic: TopicOptions = {
  name: 'markhor/diff_drive_controller/cmd_vel',
  messageType: 'geometry_msgs/Twist',
}

const joyTopic: TopicOptions = {
  name: '/joy',
  messageType: 'sensor_msgs/Joy',
}

const spaceMouseTopic: TopicOptions = {
  name: '/spacenav/twist',
  messageType: 'geometry_msgs/Twist',
}

let joySeqId = 0

const mapGamepadToJoy = (gamepad: Gamepad): IJoyMsg => {
  const d = new Date()
  const seconds = Math.round(d.getTime() / 1000)

  const lt = getBtnValue(gamepad.buttons[buttonMappings.LT])
  const rt = getBtnValue(gamepad.buttons[buttonMappings.RT])

  let axes = gamepad.axes
  axes = [-axes[0], -axes[1], lt, -axes[2], -axes[3], rt]
  const deadzone = 0.09
  axes = axes.map((x) => (x < deadzone && x > -deadzone ? 0.0 : x))

  const buttons = gamepad.buttons.map((x) => Math.floor(x.value))

  return {
    header: {
      seq: joySeqId++,
      stamp: {
        sec: seconds,
        nsecs: 0,
      },
      frame_id: '',
    },
    axes,
    buttons,
  }
}

const mapToTwist = (
  horizontal: number,
  vertical: number,
  rt: number,
  lt: number
): ITwistMsg => {
  const deadzone = 0.15
  // prettier-ignore
  const x = horizontal > deadzone
    ? -1
    : horizontal < -deadzone ? 1 : 0
  // prettier-ignore
  const y = vertical > deadzone
    ? 1
    : vertical < -deadzone ? -1 : 0

  if (lt > 0.1) {
    // brake!
    return { linear: Vector3.zero(), angular: Vector3.zero() }
  }

  return {
    linear: new Vector3(y * rt, 0, 0),
    angular: new Vector3(0, 0, x * rt),
  }
}

const getBtnValue = (rawBtn: GamepadButton) => {
  if (!rawBtn) {
    return 0
  }
  return typeof rawBtn == 'number' ? rawBtn : rawBtn.value
}

const defaultActions: Action[] = [
  {
    name: 'estop',
    bindings: [
      { type: 'keyboard', code: 'Space', onKeyDown: true },
      { type: 'gamepadBtn', button: buttonMappings.XBOX },
    ],
    perform: (ctx) => {
      // TODO use redux to toggle the estop and the related UI elements
      // This only disables the drives, if you want to restart it you need to use the UI
      if (controlService.state.matches('nothing') && ctx.type === 'keyboard') {
        return
      }
      rosClient
        .callService({ name: 'markhor/estop_disable' })
        .catch(console.error)
    },
  },
  {
    name: 'toggleArmControl',
    bindings: [{ type: 'gamepadBtn', button: buttonMappings.dpad.right }],
    perform: () => {
      controlService.send('TOGGLE')
    },
  },
  {
    name: 'flipperFront',
    bindings: [
      { type: 'gamepadBtn', button: buttonMappings.dpad.up },
      { type: 'keyboard', code: 'KeyI' },
    ],
    perform: () => {
      flipperService.send('MODE_FRONT')
    },
  },
  {
    name: 'flipperBack',
    bindings: [
      { type: 'gamepadBtn', button: buttonMappings.dpad.down },
      { type: 'keyboard', code: 'KeyK' },
    ],
    perform: () => {
      flipperService.send('MODE_BACK')
    },
  },
  {
    name: 'switchForwardDirection',
    bindings: [
      { type: 'gamepadBtn', button: buttonMappings.back },
      { type: 'keyboard', code: 'KeyT', onKeyDown: true },
    ],
    perform: () => {
      // TODO implement this client side by flipping the necessary axis direction see issue #82
      rosClient
        .callService({ name: 'markhor/switch_direction' })
        .catch(console.error)
      store.dispatch(feedSlice.actions.switchDirection())
    },
  },
  {
    name: 'headlights',
    bindings: [{ type: 'gamepadBtn', button: buttonMappings.Y }],
    perform: () => {
      rosClient.callService({ name: '/headlights' }).catch(console.error)
    },
  },
  {
    name: 'flipper_reset',
    bindings: [{ type: 'gamepadBtn', button: buttonMappings.B }],
    perform: () => {
      rosClient
        .callService({ name: 'markhor/flipper_reset' })
        .catch(console.error)
    },
  },
  {
    name: 'movement',
    bindings: [{ type: 'gamepad' }],
    perform: (ctx) => {
      if (ctx.type !== 'gamepad') {
        return
      }
      if (!controlService.state.matches('flipper')) {
        return
      }

      const {
        gamepad: { axes, buttons: btns },
      } = ctx.gamepadState

      const twist = mapToTwist(
        axes[sticks.left.horizontal],
        axes[sticks.left.vertical],
        Math.pow(getBtnValue(btns[buttonMappings.RT]), 2),
        getBtnValue(btns[buttonMappings.LT])
      )

      rosClient.publish(cmdVelTopic, twist)
    },
  },
  {
    name: 'spacemouse',
    bindings: [{ type: 'spacemouse' }],
    perform: (ctx) => {
      if (ctx.type !== 'spacemouse') {
        return
      }
      if (!controlService.state.matches('arm')) {
        return
      }

      const joy = mapGamepadToJoy(ctx.gamepadState.gamepad)
      rosClient.publish(spaceMouseTopic, joy)
    },
  },
  {
    name: 'gamepad',
    bindings: [{ type: 'gamepad' }],
    perform: (ctx) => {
      if (ctx.type !== 'gamepad') {
        return
      }
      const joy = mapGamepadToJoy(ctx.gamepadState.gamepad)
      rosClient.publish(joyTopic, joy)
    },
  },
]

const inputSystem = new InputSystem(defaultActions)

export default inputSystem
