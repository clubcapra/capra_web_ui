import CustomGamepad from './CustomGamepad'

class GamepadManager {
  gamepads: Array<CustomGamepad> = []

  constructor() {
    if (!(navigator.getGamepads instanceof Function))
      console.warn('This browser does not support gamepads.')

    this.scangamepads()

    this.update()
  }

  scangamepads() {
    const navigatorGamepads = [...navigator.getGamepads()]

    navigatorGamepads.forEach(gamepad => {
      if (gamepad === null) return
      this.gamepads[gamepad.index] = new CustomGamepad(gamepad)
    })
  }

  handleConnected = (e: GamepadEvent) => {
    const gamepad = e.gamepad
    this.gamepads[gamepad.index] = new CustomGamepad(gamepad)
  }

  handleDisconnected = (e: GamepadEvent) => {
    const gamepad = e.gamepad
    delete this.gamepads[gamepad.index]
  }

  /**
   * TODO add support for listeners
   * @param gamepad
   */
  handleGamepadInput(gamepad: CustomGamepad) {
    if (gamepad.getButtonPressed('a')) {
      console.log('a pressed')
    }

    const axis = gamepad.getAxis('left stick x')
    // console.log(axis)
  }

  update = () => {
    this.scangamepads()
    const gamepad = this.gamepads[0]
    if (gamepad) this.handleGamepadInput(gamepad)

    requestAnimationFrame(this.update)
  }
}

const instance = new GamepadManager()

window.addEventListener(
  'gamepadconnected',
  instance.handleConnected as EventListener
)
window.addEventListener(
  'gamepaddisconnected',
  instance.handleDisconnected as EventListener
)

export default instance
