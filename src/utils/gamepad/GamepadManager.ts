import { mapGamepadToTwist } from '@/utils/math/index'
import RosClient from '@/utils/ros/RosClient'
import { TopicOptions } from '@/utils/ros/types'
import CustomGamepad from './CustomGamepad'
import { GamepadBtn } from './mappings/types'

export default class GamepadManager {
  private gamepads: Array<CustomGamepad> = []

  constructor() {
    if (!(navigator.getGamepads instanceof Function))
      console.warn('This browser does not support gamepads.')

    this.initEventListeners()
    this.update()
  }

  get gamepad() {
    return this.gamepads[0]
  }

  private initEventListeners() {
    window.addEventListener('gamepadconnected', this
      .onGamepadConnected as EventListener)
    window.addEventListener('gamepaddisconnected', this
      .onGamepadDisconnected as EventListener)
  }

  // TODO add support for listeners
  private handleGamepadInput(gamepad: CustomGamepad) {
    if (gamepad.getButtonPressed(GamepadBtn.A)) {
      console.log('a pressed')
    }

    const topic: TopicOptions = {
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist',
    }

    RosClient.publish(topic, mapGamepadToTwist(gamepad))
  }

  private scanGamepads() {
    this.gamepads = [...navigator.getGamepads()]
      .filter((g): g is Gamepad => Boolean(g))
      .map(g => new CustomGamepad(g))
  }

  private update() {
    this.scanGamepads()

    const gamepad = this.gamepads[0]
    if (gamepad) this.handleGamepadInput(gamepad)

    requestAnimationFrame(this.update)
  }

  private onGamepadConnected(e: GamepadEvent) {
    const gamepad = e.gamepad
    this.gamepads[gamepad.index] = new CustomGamepad(gamepad)
  }

  private onGamepadDisconnected(e: GamepadEvent) {
    const gamepad = e.gamepad
    delete this.gamepads[gamepad.index]
  }
}
