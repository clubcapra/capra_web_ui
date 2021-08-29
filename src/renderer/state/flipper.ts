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
        console.log('set_mode_none')
      },
      set_mode_front: () => {
        console.log('set_mode_front')
      },
      set_mode_back: () => {
        console.log('set_mode_back')
      },
    },
  }
)

export const flipperService = interpret(flipperMachine).start()
