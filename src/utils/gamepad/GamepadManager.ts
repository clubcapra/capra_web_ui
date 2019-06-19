import { mapGamepadToTwist } from '@/utils/math/index'
import RosClient from '@/utils/ros/RosClient'
import { TopicOptions } from '@/utils/ros/types'
import CustomGamepad from './CustomGamepad'
import { GamepadBtn } from './mappings/types'

const topic: TopicOptions = {
  name: '/cmd_vel',
  messageType: 'geometry_msgs/Twist',
}

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

  private initEventListeners = () => {
    window.addEventListener('gamepadconnected', this
      .onGamepadConnected as EventListener)
    window.addEventListener('gamepaddisconnected', this
      .onGamepadDisconnected as EventListener)
  }

  // TODO add support for listeners
  private handleGamepadInput(gamepad: CustomGamepad) {
    if (gamepad.getButtonPressed(GamepadBtn.A)) {
      RosClient.publish(topic, mapGamepadToTwist(gamepad))
    }
  }

  private scanGamepads() {
    return [...navigator.getGamepads()]
      .filter((g): g is Gamepad => Boolean(g))
      .map(g => new CustomGamepad(g))
  }

  private update = () => {
    this.gamepads = this.scanGamepads()

    const gamepad = this.gamepads[0]
    if (gamepad) this.handleGamepadInput(gamepad)

    requestAnimationFrame(this.update)
  }

  private onGamepadConnected = (e: GamepadEvent) => {
    this.gamepads = this.scanGamepads()
  }

  private onGamepadDisconnected = (e: GamepadEvent) => {
    delete this.gamepads[e.gamepad.index]
  }
}
