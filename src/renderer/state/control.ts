import { Machine, interpret } from 'xstate';

// eslint-disable-next-line @typescript-eslint/no-empty-interface
interface ControlContext {}

interface ControlStateSchema {
  states: {
    arm: Record<string, unknown>;
    flipper: Record<string, unknown>;
    nothing: Record<string, unknown>;
  };
}

type ControlEvent =
  | { type: 'CONTROL_ARM' }
  | { type: 'CONTROL_FLIPPER' }
  | { type: 'CONTROL_NOTHING' }
  | { type: 'TOGGLE' };

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
        CONTROL_NOTHING: 'nothing',
      },
    },
    flipper: {
      on: {
        CONTROL_ARM: 'arm',
        TOGGLE: 'arm',
        CONTROL_NOTHING: 'nothing',
      },
    },
    nothing: {
      on: {
        CONTROL_ARM: 'arm',
        CONTROL_FLIPPER: 'flipper',
      },
    },
  },
});

export const controlService = interpret(controlMachine).start();
