import { Machine, interpret } from 'xstate'

// eslint-disable-next-line @typescript-eslint/no-empty-interface
interface FlipperContext {}

interface FlipperStateSchema {
  states: {
    front: Record<string, unknown>
    back: Record<string, unknown>
  }
}

type FlipperEvent = { type: 'MODE_FRONT' } | { type: 'MODE_BACK' }

export const flipperMachine = Machine<
  FlipperContext,
  FlipperStateSchema,
  FlipperEvent
>({
  id: 'flipper',
  initial: 'front',
  context: {},
  states: {
    front: {
      on: {
        MODE_BACK: 'back',
      },
    },
    back: {
      on: {
        MODE_FRONT: 'front',
      },
    },
  },
})

export const flipperService = interpret(flipperMachine).start()
