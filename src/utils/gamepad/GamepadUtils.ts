export const getButtonPressed = (button: GamepadButton) => {
  return typeof button == 'number' ? button > 0.1 : button.pressed
}

export const getButtonValue = (button: GamepadButton) => {
  return typeof button == 'number' ? button : button.value
}

export const getButton = (gamepad: Gamepad, index: number) => {
  return gamepad.buttons[index]
}
