import {
  mapGamepadToJoy,
  mapToTwist,
  cmdVelTopic,
  joyTopic,
  spaceMouseTopic,
  mapSpaceMouseToTwist,
} from './RosGamepadUtils'
import { rosClient } from 'utils/ros/rosClient'
import { store } from 'store/store'
import {
  gamepadSlice,
  isArmControlledSelector,
} from 'store/modules/gamepad/reducer'
import { GamepadBtn, Dpad, GamepadData, Stick } from 'utils/gamepad/@types'
import { isSpaceMouse, getDataFunctions } from 'utils/gamepad/GamepadUtils'

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
  const { isButtonDown } = getDataFunctions(gamepadData)

  if (isButtonDown(Dpad.Right)) {
    store.dispatch(gamepadSlice.actions.toggleIsArmControlled())
    return
  }

  if (isArmControlledSelector(store.getState())) {
    handleArmControl(gamepadData)
  } else {
    handleRobotControl(gamepadData)
  }

  if (isButtonDown(Dpad.Left)) {
    rosClient.callService({ name: '/headlights', serviceType: '' }, '')
  }
}

function handleRobotControl(data: GamepadData): void {
  const { isButtonPressed, getStick, getButtonValue } = getDataFunctions(data)

  if (isButtonPressed(GamepadBtn.A)) {
    const { horizontal, vertical } = getStick(Stick.Left)
    rosClient.publish(
      cmdVelTopic,
      mapToTwist(
        horizontal,
        vertical,
        getButtonValue(GamepadBtn.RT),
        getButtonValue(GamepadBtn.LT)
      )
    )
  }
}

function handleArmControl({ gamepad }: GamepadData): void {
  rosClient.publish(joyTopic, mapGamepadToJoy(gamepad))
}
