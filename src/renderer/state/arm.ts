import { Machine, interpret } from 'xstate'
import { sticks } from '../inputSystem/mappings'

export interface ArmContext {
  jointValue: number
  axis: number
}

interface ArmStateSchema {
  states: {
    joint: Record<string, unknown>
    cartesian: Record<string, unknown>
  }
}

type ArmEvent =
  | { type: 'MODE_JOINT' }
  | { type: 'MODE_CARTESIAN' }
  | { type: 'INCREMENT_JOINT' }
  | { type: 'DECREMENT_JOINT' }

const context: ArmContext = {
  jointValue: 0,
  axis: sticks.left.horizontal,
}

// Array that defines which axis each joint is mapped to
const armJointMappings = [
  sticks.left.horizontal, // Joint 1 left/right
  sticks.left.vertical, // Joint 2 up/down
  sticks.left.vertical, // Joint 3 up/down
  sticks.left.horizontal, // Joint 4 left/right
  sticks.left.vertical, // Joint 5 up/down
  sticks.left.horizontal, // Joint 6 left/right
]

export const armMachine = Machine<ArmContext, ArmStateSchema, ArmEvent>(
  {
    id: 'arm',
    initial: 'joint',
    context: context,
    states: {
      joint: {
        on: {
          MODE_CARTESIAN: { target: 'cartesian' },
          INCREMENT_JOINT: { target: 'joint', actions: 'increment_joint' },
          DECREMENT_JOINT: { target: 'joint', actions: 'decrement_joint' },
        },
      },
      cartesian: {
        on: {
          MODE_JOINT: { target: 'joint' },
        },
      },
    },
  },
  {
    actions: {
      decrement_joint: () => {
        if (context.jointValue < 6 && context.jointValue > 0) {
          context.jointValue--
          context.axis = armJointMappings[context.jointValue]
        }
      },
      increment_joint: () => {
        if (context.jointValue < 5 && context.jointValue >= 0) {
          context.jointValue++
          context.axis = armJointMappings[context.jointValue]
        }
      },
    },
  }
)

export const armService = interpret(armMachine).start()
