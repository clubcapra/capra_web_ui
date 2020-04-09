import { Machine, interpret } from 'xstate'

// eslint-disable-next-line @typescript-eslint/no-empty-interface
interface ControlContext {}

interface ControlStateSchema {
  states: {
    arm: {}
    flipper: {}
  }
}

type ControlEvent =
  | { type: 'CONTROL_ARM' }
  | { type: 'CONTROL_FLIPPER' }
  | { type: 'TOGGLE' }

export const controlMachine = Machine<
  ControlContext,
  ControlStateSchema,
  ControlEvent
>({
  id: 'gamepad',
  initial: 'flipper',
  context: {},
  states: {
    arm: {
      on: {
        CONTROL_FLIPPER: { target: 'flipper' },
        TOGGLE: { target: 'flipper' },
      },
    },
    flipper: {
      on: {
        CONTROL_ARM: { target: 'arm' },
        TOGGLE: { target: 'arm' },
      },
    },
  },
})

export const controlService = interpret(controlMachine).start()
