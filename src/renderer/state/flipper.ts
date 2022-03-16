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

type FlipperEvent = { type: 'MODE_FRONT' } | { type: 'MODE_BACK' }

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
          MODE_FRONT: { target: 'frontLeft', actions: 'set_mode_frontLeft' },
          MODE_BACK: { target: 'none', actions: 'set_mode_none' },
        },
      },
      frontLeft: {
        on: {
          MODE_FRONT: { target: 'frontRight', actions: 'set_mode_frontRight' },
          MODE_BACK: { target: 'front', actions: 'set_mode_front' },
        },
      },
      frontRight: {
        on: {
          MODE_FRONT: { target: 'backRight', actions: 'set_mode_backRight' },
          MODE_BACK: { target: 'frontLeft', actions: 'set_mode_frontLeft' },
        },
      },
      back: {
        on: {
          MODE_FRONT: { target: 'none', actions: 'set_mode_none' },
          MODE_BACK: { target: 'backLeft', actions: 'set_mode_backLeft' },
        },
      },
      backLeft: {
        on: {
          MODE_FRONT: { target: 'back', actions: 'set_mode_back' },
          MODE_BACK: { target: 'backRight', actions: 'set_mode_backRight' },
        },
      },
      backRight: {
        on: {
          MODE_FRONT: { target: 'backLeft', actions: 'set_mode_backLeft' },
          MODE_BACK: { target: 'frontRight', actions: 'set_mode_frontRight' },
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
    //TODO ADD INDIVIDUAL FLIPPER ACTIONS
    actions: {
      set_mode_none: () => {
        void sendFlipperMode('front_disable')
        void sendFlipperMode('back_disable')
      },
      set_mode_front: () => {
        void sendFlipperMode('front_enable')
        void sendFlipperMode('back_disable')
      },
      set_mode_back: () => {
        void sendFlipperMode('front_disable')
        void sendFlipperMode('back_enable')
      },
    },
  }
)

type FlipperMode =
  | 'front_enable'
  | 'front_disable'
  | 'back_enable'
  | 'back_disable'

async function sendFlipperMode(mode: FlipperMode) {
  try {
    await rosClient.callService({
      name: `/markhor/flipper_mode_${mode}`,
    })
  } catch (e) {
    log.error(e)
  }
}

export const flipperService = interpret(flipperMachine).start()
