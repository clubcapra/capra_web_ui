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
>({
  id: 'flipper',
  initial: 'none',
  context: {},
  states: {
    front: {
      on: {
        MODE_BACK: 'none',
      },
    },
    back: {
      on: {
        MODE_FRONT: 'none',
      },
    },
    none: {
      on: {
        MODE_FRONT: 'front',
        MODE_BACK: 'back',
      },
    },
  },
})

export const flipperService = interpret(flipperMachine).start()
