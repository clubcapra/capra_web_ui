import { log } from '@/renderer/logger'
import { Machine, interpret } from 'xstate'

export interface ArmContext {
  jointValue: number
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

const context: ArmContext = { jointValue: 0 }

export const armMachine = Machine<ArmContext, ArmStateSchema, ArmEvent>(
  {
    id: 'arm',
    initial: 'cartesian',
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
        if (context.jointValue < 5 && context.jointValue > 0) {
          context.jointValue--
        }
      },
      increment_joint: () => {
        if (context.jointValue < 5 && context.jointValue > 0) {
          context.jointValue++
        }
      },
    },
  }
)

export const armService = interpret(armMachine).start()
