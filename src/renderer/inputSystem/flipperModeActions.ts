import { buttons as buttonMappings } from '@/renderer/inputSystem/mappings';
import { rosClient } from '@/renderer/utils/ros/rosClient';
import { Action } from '@/renderer/inputSystem/@types';
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types';
import { IJoyMsg } from '@/renderer/utils/ros/rosMsgs.types';
import { store } from '@/renderer/store/store';
import { flipperService } from '@/renderer/state/flipper';
import { log } from '@/renderer/logger';
import { inputSlice, selectReverse } from '@/renderer/store/modules/input';
import { handleTpvControl } from './tpvControl';

const joyTopic: TopicOptions = {
  name: '/joy',
  messageType: 'sensor_msgs/Joy',
};

let joySeqId = 0;

const mapGamepadToJoy = (gamepad: Gamepad): IJoyMsg => {
  const d = new Date();
  const seconds = Math.round(d.getTime() / 1000);

  const lt = getBtnValue(gamepad.buttons[buttonMappings.LT]);
  const rt = getBtnValue(gamepad.buttons[buttonMappings.RT]);

  let axes = gamepad.axes;
  const isReverse = selectReverse(store.getState());
  //Disable flipper movement when TPV is active
  const rightStickEnabled = !gamepad.buttons[buttonMappings.LB].pressed;
  axes = [
    -axes[0],
    isReverse ? axes[1] : -axes[1],
    lt,
    rightStickEnabled ? -axes[2] : 0,
    rightStickEnabled ? -axes[3] : 0,
    rt,
  ];
  const deadzone = 0.15;
  axes = axes.map((x) => (x < deadzone && x > -deadzone ? 0.0 : x));
  const buttons = gamepad.buttons.map((x) => Math.floor(x.value));
  return {
    header: {
      seq: joySeqId++,
      stamp: {
        sec: seconds,
        nsecs: 0,
      },
      frame_id: '',
    },
    axes,
    buttons,
  };
};

const getBtnValue = (rawBtn: GamepadButton) =>
  typeof rawBtn == 'number' ? rawBtn : rawBtn.value;

export const flipperModeActions: Action[] = [
  {
    name: 'modeFront',
    bindings: [
      { type: 'gamepadBtnDown', button: buttonMappings.dpad.up },
      // { type: 'keyboard', code: 'KeyI' },
    ],
    perform: () => {
      const isReverse = selectReverse(store.getState());
      isReverse
        ? flipperService.send('MODE_REAR')
        : flipperService.send('MODE_FRONT');
    },
  },
  {
    name: 'modeBack',
    bindings: [
      { type: 'gamepadBtnDown', button: buttonMappings.dpad.down },
      // { type: 'keyboard', code: 'KeyK' },
    ],
    perform: () => {
      const isReverse = selectReverse(store.getState());
      isReverse
        ? flipperService.send('MODE_FRONT')
        : flipperService.send('MODE_REAR');
    },
  },
  {
    name: 'modeRight',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.dpad.right }],
    perform: () => {
      const isReverse = selectReverse(store.getState());
      isReverse
        ? flipperService.send('MODE_LEFT')
        : flipperService.send('MODE_RIGHT');
    },
  },
  {
    name: 'modeLeft',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.dpad.left }],
    perform: () => {
      const isReverse = selectReverse(store.getState());
      isReverse
        ? flipperService.send('MODE_RIGHT')
        : flipperService.send('MODE_LEFT');
    },
  },
  {
    name: 'switchForwardDirection',
    bindings: [
      { type: 'gamepadBtnDown', button: buttonMappings.Y },
      // { type: 'keyboard', code: 'KeyT', onKeyDown: true },
    ],
    perform: () => {
      /*
      Disabled for now since the TPV is used as main camera in forward and reverse modes
      store.dispatch(feedSlice.actions.switchDirection())
      */
      store.dispatch(inputSlice.actions.toggleReverse());
    },
  },

  {
    name: 'gamepad',
    bindings: [{ type: 'gamepad' }],
    perform: (ctx) => {
      if (ctx.type !== 'gamepad') {
        return;
      }

      const gamepad = ctx.gamepadState.gamepad;
      const joy = mapGamepadToJoy(ctx.gamepadState.gamepad);
      rosClient.publish(joyTopic, joy);

      handleTpvControl(gamepad);
    },
  },
];
