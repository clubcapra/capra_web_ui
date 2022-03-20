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
    back: Record<string, unknown>
    backLeft: Record<string, unknown>
    backRight: Record<string, unknown>
    none: Record<string, unknown>
  }
}

type FlipperEvent =
  | { type: 'MODE_FRONT' }
  | { type: 'MODE_BACK' }
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
          MODE_LEFT: { target: 'frontLeft', actions: 'set_mode_frontLeft' },
          MODE_RIGHT: { target: 'frontRight', actions: 'set_mode_frontRight' },
          MODE_BACK: { target: 'none', actions: 'set_mode_none' },
        },
      },
      frontLeft: {
        on: {
          MODE_RIGHT: { target: 'front', actions: 'set_mode_front' },
          MODE_BACK: { target: 'none', actions: 'set_mode_none' },
        },
      },
      frontRight: {
        on: {
          MODE_LEFT: { target: 'front', actions: 'set_mode_front' },
          MODE_BACK: { target: 'none', actions: 'set_mode_none' },
        },
      },
      back: {
        on: {
          MODE_FRONT: { target: 'none', actions: 'set_mode_none' },
          MODE_LEFT: { target: 'backLeft', actions: 'set_mode_backLeft' },
          MODE_RIGHT: { target: 'backRight', actions: 'set_mode_backRight' },
        },
      },
      backLeft: {
        on: {
          MODE_FRONT: { target: 'none', actions: 'set_mode_none' },
          MODE_RIGHT: { target: 'back', actions: 'set_mode_back' },
        },
      },
      backRight: {
        on: {
          MODE_FRONT: { target: 'none', actions: 'set_mode_none' },
          MODE_LEFT: { target: 'back', actions: 'set_mode_back' },
        },
      },
      none: {
        on: {
          MODE_FRONT: { target: 'front', actions: 'set_mode_front' },
          MODE_BACK: { target: 'back', actions: 'set_mode_back' },
        },
      },
    },
  },
  {
    actions: {
      set_mode_none: () => {
        void sendFlipperMode('front_disable')
        void sendFlipperMode('back_disable')
      },
      set_mode_front: () => {
        void sendFlipperMode('front_enable')
      },
      set_mode_frontLeft: () => {
        void sendFlipperMode('fr_disable')
      },
      set_mode_frontRight: () => {
        void sendFlipperMode('fl_disable')
      },
      set_mode_back: () => {
        void sendFlipperMode('back_enable')
      },
      set_mode_backLeft: () => {
        void sendFlipperMode('rr_disable')
      },
      set_mode_backRight: () => {
        void sendFlipperMode('rl_disable')
      },
    },
  }
)

type FlipperMode =
  | 'front_enable'
  | 'front_disable'
  | 'back_enable'
  | 'back_disable'
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
