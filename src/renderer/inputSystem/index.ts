import { InputSystem } from '@/renderer/inputSystem/InputSystem'
import { buttons as buttonMappings } from '@/renderer/inputSystem/mappings'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import { Action } from '@/renderer/inputSystem/@types'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import { IJoyMsg } from '@/renderer/utils/ros/rosMsgs.types'
import { controlService } from '@/renderer/state/control'
import { feedSlice } from '@/renderer/store/modules/feed'
import { store } from '@/renderer/store/store'
import { flipperService } from '@/renderer/state/flipper'
import { log } from '@/renderer/logger'
import { inputSlice, selectReverse } from '@/renderer/store/modules/input'
import { armService } from '../state/arm'
import { useActor } from '@xstate/react'

const joyTopic: TopicOptions = {
  name: '/joy',
  messageType: 'sensor_msgs/Joy',
}

const spaceMouseTopic: TopicOptions = {
  name: '/spacenav/twist',
  messageType: 'geometry_msgs/Twist',
}

const jointGoalTopic: TopicOptions = {
  name: 'ovis/joint_goal',
  messageType: 'ovis_msgs/OvisJointGoal',
}

let joySeqId = 0

const mapGamepadToJoy = (gamepad: Gamepad): IJoyMsg => {
  const d = new Date()
  const seconds = Math.round(d.getTime() / 1000)

  const lt = getBtnValue(gamepad.buttons[buttonMappings.LT])
  const rt = getBtnValue(gamepad.buttons[buttonMappings.RT])

  let axes = gamepad.axes
  const isReverse = selectReverse(store.getState())
  axes = [-axes[0], isReverse ? axes[1] : -axes[1], lt, -axes[2], -axes[3], rt]
  const deadzone = 0.15
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

const getBtnValue = (rawBtn: GamepadButton) =>
  typeof rawBtn == 'number' ? rawBtn : rawBtn.value

/*const getAxesValue = (rawAxes: number[]) => {
  if (!controlService.state.matches('arm')) {

  }
}*/

const defaultActions: Action[] = [
  {
    name: 'estop',
    bindings: [
      { type: 'keyboard', code: 'Space', onKeyDown: true },
      { type: 'gamepadBtnDown', button: buttonMappings.XBOX },
    ],
    perform: (ctx) => {
      // TODO use redux to toggle the estop and the related UI elements
      // This only disables the drives, if you want to restart it you need to use the UI
      if (controlService.state.matches('nothing') && ctx.type === 'keyboard') {
        return
      }
      rosClient.callService({ name: 'markhor/estop_disable' }).catch(log.error)
    },
  },
  {
    name: 'toggleArmControl',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.start }],
    perform: () => {
      controlService.send('TOGGLE')
    },
  },
  {
    name: 'flipperFront',
    bindings: [
      { type: 'gamepadBtnDown', button: buttonMappings.dpad.up },
      // { type: 'keyboard', code: 'KeyI' },
    ],
    perform: () => {
      if (controlService.state.matches('flipper')) {
        flipperService.send('MODE_FRONT')
      } else {
        armService.send('MODE_CARTESIAN')
      }
    },
  },
  {
    name: 'flipperBack',
    bindings: [
      { type: 'gamepadBtnDown', button: buttonMappings.dpad.down },
      // { type: 'keyboard', code: 'KeyK' },
    ],
    perform: () => {
      if (controlService.state.matches('flipper')) {
        flipperService.send('MODE_REAR')
      } else {
        armService.send('MODE_JOINT')
      }
    },
  },
  {
    name: 'flipperRight',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.dpad.right }],
    perform: () => {
      flipperService.send('MODE_RIGHT')
    },
  },
  {
    name: 'flipperLeft',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.dpad.left }],
    perform: () => {
      flipperService.send('MODE_LEFT')
    },
  },
  {
    name: 'switchForwardDirection',
    bindings: [
      { type: 'gamepadBtnDown', button: buttonMappings.Y },
      // { type: 'keyboard', code: 'KeyT', onKeyDown: true },
    ],
    perform: () => {
      // TODO implement this client side by flipping the necessary axis direction see issue #82
      // rosClient
      //   .callService({ name: 'markhor/switch_direction' })
      //   .catch(log.error)
      store.dispatch(feedSlice.actions.switchDirection())
      store.dispatch(inputSlice.actions.toggleReverse())
    },
  },
  {
    name: 'headlights',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.X }],
    perform: () => {
      rosClient.callService({ name: '/headlights' }).catch(log.error)
    },
  },
  {
    name: 'flipper_reset',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.B }],
    perform: () => {
      rosClient
        .callService({ name: 'markhor/flippers/flipper_reset' })
        .catch(log.error)
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

      const gamepad = ctx.gamepadState.gamepad
      if (!controlService.state.matches('arm')) {
        rosClient.publish(jointGoalTopic, {
          joint_index: armService.state.context.jointValue,
          joint_angle: gamepad.axes[0],
        })
      }

      const joy = mapGamepadToJoy(ctx.gamepadState.gamepad)
      const tpvEnabled = gamepad.buttons[buttonMappings.LB].pressed
      rosClient.publish(joyTopic, joy)

      //rosClient.publish(tpvXTopic, tpvEnabled ? gamepad.axes[2] : 0)
      //rosClient.publish(tpvYTopic, tpvEnabled ? gamepad.axes[3] : 0)
    },
  },
]

const inputSystem = new InputSystem(defaultActions)

export default inputSystem
