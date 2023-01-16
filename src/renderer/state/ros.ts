import { Machine, assign, interpret } from 'xstate';
import { rosClient } from '@/renderer/utils/ros/rosClient';
import { toast } from 'react-toastify';
import {
  selectFullAddress,
  selectIP,
  selectPort,
} from '@/renderer/store/modules/ros';
import { store } from '@/renderer/store/store';
import { log } from '@/renderer/logger';

interface RosContext {
  connectingToastId: string;
  errorToastId: string;
}

interface RosStateSchema {
  states: {
    connected: Record<string, unknown>;
    connecting: Record<string, unknown>;
    disconnected: Record<string, unknown>;
  };
}

type RosEvent =
  | { type: 'DISCONNECT' }
  | { type: 'CONNECT' }
  | { type: 'FAIL' }
  | { type: 'SUCCESS' };

export const rosMachine = Machine<RosContext, RosStateSchema, RosEvent>(
  {
    id: 'ros',
    initial: 'disconnected',
    context: {
      connectingToastId: '',
      errorToastId: '',
    },
    states: {
      connected: {
        on: {
          DISCONNECT: {
            target: 'disconnected',
            actions: ['toastDisconnect', 'rosClientDisconnect'],
          },
        },
      },
      connecting: {
        entry: ['rosClientConnect', 'toastConnecting'],
        on: {
          SUCCESS: { target: 'connected', actions: 'toastSuccess' },
          FAIL: { target: 'disconnected', actions: 'toastFail' },
        },
      },
      disconnected: {
        on: {
          CONNECT: 'connecting',
        },
      },
    },
  },
  {
    actions: {
      rosClientConnect: () => {
        const state = store.getState();
        const IP = selectIP(state);
        const port = selectPort(state);
        rosClient.connect(IP, port);
      },
      rosClientDisconnect: () => {
        rosClient.disconnect();
      },
      toastSuccess: (ctx) => {
        const state = store.getState();
        toast.dismiss(ctx.connectingToastId);
        toast.info(`ROS: Connected to: ${selectFullAddress(state)}`);
      },
      toastFail: (ctx) => {
        const state = store.getState();
        toast.dismiss(ctx.connectingToastId);
        const message = `ROS: Failed to connect to: ${selectFullAddress(
          state
        )}`;
        log.error(message);
        const id = toast.error(message);
        assign({ errorToastId: id });
      },
      toastConnecting: (ctx) => {
        const state = store.getState();
        toast.dismiss(ctx.errorToastId);
        toast.dismiss(ctx.connectingToastId);
        const id = toast.warn(
          `ROS: Trying to connect to: ${selectFullAddress(state)}`
        );
        assign({ connectingToastId: id });
      },
      toastDisconnect: (ctx) => {
        const state = store.getState();
        toast.dismiss(ctx.connectingToastId);
        toast.warn(`ROS: Lost connection to: ${selectFullAddress(state)}`);
      },
    },
  }
);

export const rosService = interpret(rosMachine).start();
