import { mapGamepadToTwist } from '@/utils/math/index'
import RosClient from '@/utils/ros/RosClient'
import { TopicOptions } from '@/utils/ros/types'
import CustomGamepad from './CustomGamepad'
import { GamepadBtn, Dpad } from './mappings/types'
import { mapGamepadToJoy } from './GamepadUtils'

const cmdVelTopic: TopicOptions = {
  name: '/cmd_vel',
  messageType: 'geometry_msgs/Twist',
}

const joyTopic: TopicOptions = {
  name: '/joy',
  messageType: 'sensor_msgs/Joy',
}

export default class GamepadManager {
  private gamepads: Array<CustomGamepad> = []
  private headlightsOn: Boolean = false
  private armTogglePressed: Boolean = false
  private isArmControlled: Boolean = false

  constructor() {
    if (!(navigator.getGamepads instanceof Function))
      console.warn('This browser does not support gamepads.')

    this.initEventListeners()
    this.update()
  }

  get getIsArmControlled() {
    return this.isArmControlled
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
    this.handleControlMode(gamepad)

    if (this.isArmControlled) {
      this.handleArmControl(gamepad)
    } else {
      this.handleRobotControl(gamepad)
    }

    this.handleHeadLight(gamepad)
  }

  private handleRobotControl(gamepad: CustomGamepad) {
    if (gamepad.getButtonPressed(GamepadBtn.A)) {
      RosClient.publish(cmdVelTopic, mapGamepadToTwist(gamepad))
    }
  }
  private handleArmControl(gamepad: CustomGamepad) {
    RosClient.publish(joyTopic, mapGamepadToJoy(gamepad.gamepad))
  }

  private handleControlMode(gamepad: CustomGamepad) {
    if (gamepad.getButtonPressed(Dpad.Right) && !this.armTogglePressed) {
      this.isArmControlled = !this.isArmControlled
      this.armTogglePressed = true
    } else if (!gamepad.getButtonPressed(Dpad.Right) && this.armTogglePressed) {
      this.armTogglePressed = false
    }
  }

  private handleHeadLight(gamepad: CustomGamepad) {
    if (gamepad.getButtonPressed(Dpad.Left) && this.headlightsOn) {
      RosClient.callService({ name: '/headlights', serviceType: '' }, '')
      this.headlightsOn = true
    } else if (!gamepad.getButtonPressed(Dpad.Left) && !this.headlightsOn) {
      this.headlightsOn = false
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
