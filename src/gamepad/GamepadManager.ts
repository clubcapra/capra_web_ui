import CustomGamepad from './CustomGamepad'
import { mapGamepadToTwist } from '@/utils/math'

class GamepadManager {
  gamepads: Array<CustomGamepad> = []

  constructor() {
    if (!(navigator.getGamepads instanceof Function))
      console.warn('This browser does not support gamepads.')

    this.scanGamepads()

    this.update()
  }

  scanGamepads() {
    const navigatorGamepads = [...navigator.getGamepads()]

    navigatorGamepads.forEach(gamepad => {
      if (gamepad === null) return
      this.gamepads[gamepad.index] = new CustomGamepad(gamepad)
    })
  }

  handleConnected = (e: GamepadEvent) => {
    console.log('gamepad connected', e.gamepad)
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

    const xAxis = gamepad.getAxis('left stick x')
    const yAxis = gamepad.getAxis('left stick y')
    const rightTrigger = gamepad.getButtonAxis('right trigger')
    // console.log("TCL: GamepadManager -> handleGamepadInput -> rightTrigger", rightTrigger)

    const cmd_vel = mapGamepadToTwist(xAxis, yAxis, rightTrigger ? 1 : 0);
    console.log('linear', cmd_vel.linear, 'angular', cmd_vel.angular)
  }

  update = () => {
    this.scanGamepads()
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
