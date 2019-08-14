import CustomGamepad from './CustomGamepad'
import {
  mapGamepadToJoy,
  mapGamepadToTwist,
  cmdVelTopic,
  joyTopic,
  spaceMouseTopic,
  mapSpaceMouseToTwist,
} from './RosGamepadUtils'
import { rosClient } from 'utils/ros/rosClient'
import { store } from 'store/store'
import { gamepadSlice } from 'store/modules/gamepad/reducer'
import { GamepadBtn, Dpad, InputHandler } from 'utils/gamepad/@types'
import { isSpaceMouse } from 'utils/gamepad/GamepadUtils'

export class DefaultInputHandler implements InputHandler {
  handleGamepadInput(gamepad: CustomGamepad): void {
    if (isSpaceMouse(gamepad)) {
      this.handleSpaceMouse(gamepad)
    } else {
      this.handleGamepad(gamepad)
    }
  }

  private handleSpaceMouse(spaceMouse: CustomGamepad): void {
    rosClient.publish(spaceMouseTopic, mapSpaceMouseToTwist(spaceMouse.gamepad))
  }

  private handleGamepad(gamepad: CustomGamepad): void {
    if (gamepad.getTogglePressed(Dpad.Right)) {
      store.dispatch(gamepadSlice.actions.toggleIsArmControlled)
      return
    }

    if (store.getState().gamepad.isArmControlled) {
      this.handleArmControl(gamepad)
    } else {
      this.handleRobotControl(gamepad)
    }

    if (gamepad.getTogglePressed(Dpad.Left)) {
      rosClient.callService({ name: '/headlights', serviceType: '' }, '')
    }
  }

  private handleRobotControl(gamepad: CustomGamepad): void {
    if (gamepad.getButtonPressed(GamepadBtn.A)) {
      rosClient.publish(cmdVelTopic, mapGamepadToTwist(gamepad))
    }
  }

  private handleArmControl(gamepad: CustomGamepad): void {
    rosClient.publish(joyTopic, mapGamepadToJoy(gamepad.gamepad))
  }
}
