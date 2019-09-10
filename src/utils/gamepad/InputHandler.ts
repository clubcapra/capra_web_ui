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
import { GamepadBtn, Dpad, GamepadData } from 'utils/gamepad/@types'
import {
  isSpaceMouse,
  getTogglePressed,
  getButtonPressed,
} from 'utils/gamepad/GamepadUtils'

// NOTE: This could be implemented like a reducer with the action being the event
// like button pressed or axis moved. The GamepadManager would be the one dispatching the events.
// It should be implemented in a way that you can register n reducer
// and they would all run when an event is fired
export function handleGamepadInput(gamepadData: GamepadData): void {
  if (isSpaceMouse(gamepadData)) {
    handleSpaceMouse(gamepadData)
  } else {
    handleGamepad(gamepadData)
  }
}

function handleSpaceMouse(gamepadData: GamepadData): void {
  rosClient.publish(spaceMouseTopic, mapSpaceMouseToTwist(gamepadData.gamepad))
}

function handleGamepad(gamepadData: GamepadData): void {
  const isTogglePressed = getTogglePressed(gamepadData)

  if (isTogglePressed(Dpad.Right)) {
    store.dispatch(gamepadSlice.actions.toggleIsArmControlled)
    return
  }

  if (store.getState().gamepad.isArmControlled) {
    handleArmControl(gamepadData)
  } else {
    handleRobotControl(gamepadData)
  }

  if (isTogglePressed(Dpad.Left)) {
    rosClient.callService({ name: '/headlights', serviceType: '' }, '')
  }
}

function handleRobotControl(data: GamepadData): void {
  if (getButtonPressed(GamepadBtn.A)(data.gamepad, data.mapping)) {
    rosClient.publish(cmdVelTopic, mapGamepadToTwist(data))
  }
}

function handleArmControl({ gamepad }: GamepadData): void {
  rosClient.publish(joyTopic, mapGamepadToJoy(gamepad))
}
