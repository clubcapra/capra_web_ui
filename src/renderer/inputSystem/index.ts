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
import { armModeActions } from './armModeActions'
import { flipperModeActions } from './flipperModeActions'

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
  if (controlService.state.matches('arm')) {
    axes = [0, 0, 0, 0, 0, 0]
  } else {
    const isReverse = selectReverse(store.getState())
    const rightStickEnabled = !gamepad.buttons[buttonMappings.LB].pressed
    axes = [
      -axes[0],
      isReverse ? axes[1] : -axes[1],
      lt,
      rightStickEnabled ? -axes[2] : 0,
      rightStickEnabled ? -axes[3] : 0,
      rt,
    ]
    const deadzone = 0.15
    axes = axes.map((x) => (x < deadzone && x > -deadzone ? 0.0 : x))
  }
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
      if (controlService.state.matches('arm')) {
        inputSystem.setActionMap(defaultActions.concat(armModeActions))
      } else {
        inputSystem.setActionMap(defaultActions.concat(flipperModeActions))
      }
    },
  },
]

const inputSystem = new InputSystem(defaultActions.concat(flipperModeActions))

export default inputSystem
