import { InputSystem } from '@/renderer/inputSystem/InputSystem';
import { buttons as buttonMappings } from '@/renderer/inputSystem/mappings';
import { rosClient } from '@/renderer/utils/ros/rosClient';
import { Action } from '@/renderer/inputSystem/@types';
import { controlService } from '@/renderer/state/control';
import { log } from '@/renderer/logger';
import { armModeActions } from './armModeActions';
import { flipperModeActions } from './flipperModeActions';
import { TopicOptions } from '../utils/ros/roslib-ts-client/@types';

const gripperTopic: TopicOptions = {
  name: 'ovis/gripper/position_goal',
  messageType: 'ovis_robotiq_gripper/OvisGripperPosition',
};

const defaultActions: Action[] = [
  {
    name: 'estop',
    bindings: [
      { type: 'keyboard', code: 'Space', onKeyDown: true },
      { type: 'gamepadBtnDown', button: buttonMappings.XBOX },
    ],
    perform: (ctx) => {
      // TODO use redux to toggle the estop and the related UI elements
      // This only disables the drives, if you want to restart it you need to use the UI
      if (controlService.state.matches('nothing') && ctx.type === 'keyboard') {
        return;
      }
      rosClient.callService({ name: 'markhor/estop_disable' }).catch(log.error);
    },
  },
  {
    name: 'toggleArmControl',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.start }],
    perform: () => {
      controlService.send('TOGGLE');
      if (controlService.state.matches('arm')) {
        inputSystem.setActionMap(defaultActions.concat(armModeActions));
      } else {
        inputSystem.setActionMap(defaultActions.concat(flipperModeActions));
      }
    },
  },
  // Allow gripper movement in both flipper and arm mode
  {
    name: 'toggleGripper',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.B }],
    perform: () => {
      // Toggle gripper
      rosClient.publish(gripperTopic, {
        position: 2,
      });
    },
  },

  {
    name: 'wristlights',
    bindings: [{ type: 'gamepadBtnDown', button: buttonMappings.X }],
    perform: () => {
      rosClient
        .callService({ name: '/capra/wrist_light_toggle' })
        .catch(log.error);
    },
  },
];

const inputSystem = new InputSystem(defaultActions.concat(flipperModeActions));

export default inputSystem;
