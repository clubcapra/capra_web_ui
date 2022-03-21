import { log } from '@/renderer/logger'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import { Machine, interpret } from 'xstate'

// eslint-disable-next-line @typescript-eslint/no-empty-interface
interface FlipperContext {}

interface FlipperStateSchema {
  states: {
    front: Record<string, unknown>
    frontLeft: Record<string, unknown>
    frontRight: Record<string, unknown>
    rear: Record<string, unknown>
    rearLeft: Record<string, unknown>
    rearRight: Record<string, unknown>
    none: Record<string, unknown>
  }
}

type FlipperEvent =
  | { type: 'MODE_FRONT' }
  | { type: 'MODE_REAR' }
  | { type: 'MODE_LEFT' }
  | { type: 'MODE_RIGHT' }

export const flipperMachine = Machine<
  FlipperContext,
  FlipperStateSchema,
  FlipperEvent
>(
  {
    id: 'flipper',
    initial: 'none',
    context: {},
    states: {
      front: {
        on: {
          MODE_LEFT: { target: 'frontLeft', actions: 'set_mode_fl' },
          MODE_RIGHT: { target: 'frontRight', actions: 'set_mode_fr' },
          MODE_REAR: { target: 'none', actions: 'set_mode_none' },
        },
      },
      frontLeft: {
        on: {
          MODE_RIGHT: { target: 'front', actions: 'set_mode_front' },
          MODE_REAR: { target: 'none', actions: 'set_mode_none' },
        },
      },
      frontRight: {
        on: {
          MODE_LEFT: { target: 'front', actions: 'set_mode_front' },
          MODE_REAR: { target: 'none', actions: 'set_mode_none' },
        },
      },
      rear: {
        on: {
          MODE_FRONT: { target: 'none', actions: 'set_mode_none' },
          MODE_LEFT: { target: 'rearLeft', actions: 'set_mode_rl' },
          MODE_RIGHT: { target: 'rearRight', actions: 'set_mode_rr' },
        },
      },
      rearLeft: {
        on: {
          MODE_FRONT: { target: 'none', actions: 'set_mode_none' },
          MODE_RIGHT: { target: 'rear', actions: 'set_mode_rear' },
        },
      },
      rearRight: {
        on: {
          MODE_FRONT: { target: 'none', actions: 'set_mode_none' },
          MODE_LEFT: { target: 'rear', actions: 'set_mode_rear' },
        },
      },
      none: {
        on: {
          MODE_FRONT: { target: 'front', actions: 'set_mode_front' },
          MODE_REAR: { target: 'rear', actions: 'set_mode_rear' },
        },
      },
    },
  },
  {
    actions: {
      set_mode_none: () => {
        void sendFlipperMode('front_disable')
        void sendFlipperMode('rear_disable')
      },
      set_mode_front: () => {
        void sendFlipperMode('front_enable')
      },
      set_mode_fl: () => {
        void sendFlipperMode('fr_disable')
      },
      set_mode_fr: () => {
        void sendFlipperMode('fl_disable')
      },
      set_mode_rear: () => {
        void sendFlipperMode('rear_enable')
      },
      set_mode_rl: () => {
        void sendFlipperMode('rr_disable')
      },
      set_mode_rr: () => {
        void sendFlipperMode('rl_disable')
      },
    },
  }
)

type FlipperMode =
  | 'front_enable'
  | 'front_disable'
  | 'rear_enable'
  | 'rear_disable'
  | 'fr_disable'
  | 'fl_disable'
  | 'rr_disable'
  | 'rl_disable'

async function sendFlipperMode(mode: FlipperMode) {
  try {
    await rosClient.callService({
      name: `/markhor/flippers/flipper_mode_${mode}`,
    })
  } catch (e) {
    log.error(e)
  }
}

export const flipperService = interpret(flipperMachine).start()
