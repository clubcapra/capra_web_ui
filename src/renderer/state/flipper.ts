import { log } from '@/renderer/logger';
import { rosClient } from '@/renderer/utils/ros/rosClient';
import { Machine, interpret } from 'xstate';
import { store } from '@/renderer/store/store';
import { selectRobotNameState } from '@/renderer/store/modules/input';

// eslint-disable-next-line @typescript-eslint/no-empty-interface
interface FlipperContext {}

interface FlipperStateSchema {
  states: {
    front: Record<string, unknown>;
    fl: Record<string, unknown>;
    fr: Record<string, unknown>;
    rear: Record<string, unknown>;
    rl: Record<string, unknown>;
    rr: Record<string, unknown>;
    all: Record<string, unknown>;
  };
}

type FlipperEvent =
  | { type: 'MODE_FRONT' }
  | { type: 'MODE_REAR' }
  | { type: 'MODE_LEFT' }
  | { type: 'MODE_RIGHT' };

export const flipperMachine = Machine<
  FlipperContext,
  FlipperStateSchema,
  FlipperEvent
>(
  {
    id: 'flipper',
    initial: 'all',
    context: {},
    states: {
      front: {
        on: {
          MODE_LEFT: { target: 'fl', actions: 'set_mode_fl' },
          MODE_RIGHT: { target: 'fr', actions: 'set_mode_fr' },
          MODE_REAR: { target: 'all', actions: 'set_mode_all' },
        },
      },
      fl: {
        on: {
          MODE_RIGHT: { target: 'front', actions: 'set_mode_front' },
          MODE_REAR: { target: 'all', actions: 'set_mode_all' },
        },
      },
      fr: {
        on: {
          MODE_LEFT: { target: 'front', actions: 'set_mode_front' },
          MODE_REAR: { target: 'all', actions: 'set_mode_all' },
        },
      },
      rear: {
        on: {
          MODE_FRONT: { target: 'all', actions: 'set_mode_all' },
          MODE_LEFT: { target: 'rl', actions: 'set_mode_rl' },
          MODE_RIGHT: { target: 'rr', actions: 'set_mode_rr' },
        },
      },
      rl: {
        on: {
          MODE_FRONT: { target: 'all', actions: 'set_mode_all' },
          MODE_RIGHT: { target: 'rear', actions: 'set_mode_rear' },
        },
      },
      rr: {
        on: {
          MODE_FRONT: { target: 'all', actions: 'set_mode_none' },
          MODE_LEFT: { target: 'rear', actions: 'set_mode_rear' },
        },
      },
      all: {
        on: {
          MODE_FRONT: { target: 'front', actions: 'set_mode_front' },
          MODE_REAR: { target: 'rear', actions: 'set_mode_rear' },
        },
      },
    },
  },
  {
    actions: {
      set_mode_all: () => {
        void sendFlipperMode('front_enable');
        void sendFlipperMode('rear_enable');
      },
      set_mode_front: () => {
        void sendFlipperMode('front_enable');
        void sendFlipperMode('rear_disable');
      },
      set_mode_fl: () => {
        void sendFlipperMode('fr_disable');
      },
      set_mode_fr: () => {
        void sendFlipperMode('fl_disable');
      },
      set_mode_rear: () => {
        void sendFlipperMode('rear_enable');
        void sendFlipperMode('front_disable');
      },
      set_mode_rl: () => {
        void sendFlipperMode('rr_disable');
      },
      set_mode_rr: () => {
        void sendFlipperMode('rl_disable');
      },
    },
  }
);

type FlipperMode =
  | 'front_enable'
  | 'front_disable'
  | 'rear_enable'
  | 'rear_disable'
  | 'fr_disable'
  | 'fl_disable'
  | 'rr_disable'
  | 'rl_disable';

async function sendFlipperMode(mode: FlipperMode) {
  const robotName = selectRobotNameState(store.getState());
  try {
    await rosClient.callService({
      name: `${robotName}/flippers/flipper_mode_${mode}`,
    });
  } catch (e) {
    log.error(e);
  }
}

export const flipperService = interpret(flipperMachine).start();
