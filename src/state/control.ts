import { Machine, interpret } from 'xstate'

// eslint-disable-next-line @typescript-eslint/no-empty-interface
interface ControlContext {}

interface ControlStateSchema {
  states: {
    arm: {}
    flipper: {}
    nothing: {}
  }
}

type ControlEvent =
  | { type: 'CONTROL_ARM' }
  | { type: 'CONTROL_FLIPPER' }
  | { type: 'DISABLE' }
  | { type: 'TOGGLE' }

export const controlMachine = Machine<
  ControlContext,
  ControlStateSchema,
  ControlEvent
>({
  id: 'control',
  initial: 'flipper',
  context: {},
  states: {
    arm: {
      on: {
        CONTROL_FLIPPER: 'flipper',
        TOGGLE: 'flipper',
        DISABLE: 'nothing',
      },
    },
    flipper: {
      on: {
        CONTROL_ARM: 'arm',
        TOGGLE: 'arm',
        DISABLE: 'nothing',
      },
    },
    nothing: {
      on: {
        CONTROL_ARM: 'arm',
        CONTROL_FLIPPER: 'flipper',
      },
    },
  },
})

export const controlService = interpret(controlMachine).start()
