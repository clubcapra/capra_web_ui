import CustomGamepad from './CustomGamepad'
import { mapGamepadToTwist } from '@/utils/math/index'
import RosClient from '@/utils/ros/RosClient'

export default class GamepadManager {
  gamepads: Array<CustomGamepad> = []
  ros: RosClient

  constructor(ros: RosClient) {
    this.ros = ros

    if (!(navigator.getGamepads instanceof Function))
      console.warn('This browser does not support gamepads.')

    this.initEventListeners()
    this.scanGamepads()
    this.update()
  }

  initEventListeners() {
    window.addEventListener('gamepadconnected', this
      .handleConnected as EventListener)
    window.addEventListener('gamepaddisconnected', this
      .handleDisconnected as EventListener)
  }

  scanGamepads() {
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

    const xAxis = gamepad.getAxis('left stick x')
    const yAxis = gamepad.getAxis('left stick y')
    const rightTrigger = gamepad.getButtonAxis('right trigger')

    const cmd_vel = mapGamepadToTwist(xAxis, yAxis, rightTrigger ? 1 : 0)
  }

  update = () => {
    this.scanGamepads()

    const gamepad = this.gamepads[0]
    if (gamepad) this.handleGamepadInput(gamepad)

    requestAnimationFrame(this.update)
  }
}
