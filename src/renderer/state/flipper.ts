import { rosClient } from '@/renderer/utils/ros/rosClient'
import { Machine, interpret } from 'xstate'

// eslint-disable-next-line @typescript-eslint/no-empty-interface
interface FlipperContext {}

interface FlipperStateSchema {
  states: {
    front: Record<string, unknown>
    back: Record<string, unknown>
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
          MODE_BACK: { target: 'none', actions: 'set_mode_none' },
        },
      },
      back: {
        on: {
          MODE_FRONT: { target: 'none', actions: 'set_mode_none' },
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
      name: `/markhor/markhor_base_flipper_node/flipper_mode_${mode}`,
    })
  } catch (e) {
    console.error(e)
  }
}

export const flipperService = interpret(flipperMachine).start()
