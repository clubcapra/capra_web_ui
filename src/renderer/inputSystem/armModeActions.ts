import { buttons as buttonMappings, sticks } from '@/renderer/inputSystem/mappings'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import { Action } from '@/renderer/inputSystem/@types'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import { log } from '@/renderer/logger'
import { ArmContext, armService } from '../state/arm'
import { deadzone } from '../utils/gamepad'

const jointGoalTopic: TopicOptions = {
  name: 'ovis/arm/joint_goal',
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
    name: 'openGripper',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.X }],
    perform: () => {
      log.info('open gripper')
    },
  },
  {
    name: 'closeGripper',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.B }],
    perform: () => {
      log.info('close gripper')
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
      const tpvEnabled = gamepad.buttons[buttonMappings.LB].pressed
      rosClient.publish(
        tpvXTopic,
        tpvEnabled
          ? { data: deadzone(gamepad.axes[sticks.right.horizontal]) }
          : { data: 0 }
      )
      rosClient.publish(
        tpvYTopic,
        tpvEnabled
          ? { data: deadzone(gamepad.axes[sticks.right.vertical]) }
          : { data: 0 }
      )
    },
  },
]
