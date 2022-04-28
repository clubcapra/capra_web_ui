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
import { ArmContext, armService } from '../state/arm'

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

const tpvXTopic: TopicOptions = {
  name: '/tpv_x',
  messageType: 'std_msgs/Float64',
}

const tpvYTopic: TopicOptions = {
  name: '/tpv_y',
  messageType: 'std_msgs/Float64',
}

let joySeqId = 0

const mapGamepadToJoy = (gamepad: Gamepad): IJoyMsg => {
  const d = new Date()
  const seconds = Math.round(d.getTime() / 1000)

  const lt = getBtnValue(gamepad.buttons[buttonMappings.LT])
  const rt = getBtnValue(gamepad.buttons[buttonMappings.RT])

  let axes = gamepad.axes
  axes = [0, 0, 0, 0, 0, 0]
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

export const armModeActions: Action[] = [
  {
    name: 'modeCartesian',
    bindings: [
      { type: 'gamepadBtnDown', button: buttonMappings.dpad.up },
      // { type: 'keyboard', code: 'KeyI' },
    ],
    perform: () => {
      armService.send('MODE_CARTESIAN')
    },
  },
  {
    name: 'modeJoint',
    bindings: [
      { type: 'gamepadBtnDown', button: buttonMappings.dpad.down },
      // { type: 'keyboard', code: 'KeyK' },
    ],
    perform: () => {
      armService.send('MODE_JOINT')
    },
  },
  {
    name: 'incrementJoint',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.dpad.right }],
    perform: () => {
      armService.send('INCREMENT_JOINT')
    },
  },
  {
    name: 'decrementJoint',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.dpad.left }],
    perform: () => {
      armService.send('DECREMENT_JOINT')
    },
  },
  {
    name: 'home',
    bindings: [
      { type: 'gamepadBtnDown', button: buttonMappings.Y },
      // { type: 'keyboard', code: 'KeyT', onKeyDown: true },
    ],
    perform: () => {
      console.log('home')
    },
  },
  {
    name: 'openGripper',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.X }],
    perform: () => {
      console.log('open gripper')
    },
  },
  {
    name: 'closeGripper',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.B }],
    perform: () => {
      console.log("close gripper")
    },
  },
  {
    name: 'spacemouse',
    bindings: [{ type: 'spacemouse' }],
    perform: (ctx) => {
      if (ctx.type !== 'spacemouse') {
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
      if (armService.state.matches('joint')) {
        if (gamepad.buttons[buttonMappings.A].pressed) {
          rosClient.publish(jointGoalTopic, {
            joint_index: (armService.state.context as ArmContext).jointValue,
            joint_velocity:
              gamepad.axes[1] < 0.15 && gamepad.axes[1] > -0.15
                ? 0
                : -gamepad.axes[1],
          })
        }
      }

      const joy = mapGamepadToJoy(ctx.gamepadState.gamepad)
      const tpvEnabled = gamepad.buttons[buttonMappings.LB].pressed
      rosClient.publish(joyTopic, joy)
      rosClient.publish(
        tpvXTopic,
        tpvEnabled ? { data: gamepad.axes[2] } : { data: 0 }
      )
      rosClient.publish(
        tpvYTopic,
        tpvEnabled ? { data: gamepad.axes[3] } : { data: 0 }
      )
    },
  },
]

