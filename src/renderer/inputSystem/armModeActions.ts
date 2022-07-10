import {
  buttons as buttonMappings,
  sticks,
} from '@/renderer/inputSystem/mappings'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import { Action } from '@/renderer/inputSystem/@types'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import { log } from '@/renderer/logger'
import { ArmContext, armService } from '../state/arm'
import { deadzone } from '../utils/gamepad'
import { handleTpvControl } from './tpvControl'

const jointGoalTopic: TopicOptions = {
  name: 'ovis/arm/joint_velocity_goal',
  messageType: 'ovis_msgs/OvisArmJointVelocity',
}

const gripperTopic: TopicOptions = {
  name: 'ovis/gripper/position_goal',
  messageType: 'ovis_robotiq_gripper/OvisGripperPosition',
}

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
      rosClient
        .callService({ name: '/ovis/arm/home_joint_positions' })
        .catch(log.error)
    },
  },
  {
    name: 'toggleGripper',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.B }],
    perform: () => {
      rosClient.publish(gripperTopic, {
        position: 2,
      })
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
            joint_velocity: -deadzone(gamepad.axes[sticks.left.vertical]),
          })
        }
      }
      handleTpvControl(gamepad)
    },
  },
]
