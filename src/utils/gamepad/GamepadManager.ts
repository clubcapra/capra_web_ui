import { TopicOptions } from '@/utils/ros/types'
import CustomGamepad from './CustomGamepad'
import { mapGamepadToTwist } from '@/utils/math/index'
import RosClient from '@/utils/ros/RosClient'
import { ButtonNames, StickDirections } from './mappings/types'

export default class GamepadManager {
  private gamepads: Array<CustomGamepad> = []
  private ros: RosClient

  constructor(ros: RosClient) {
    this.ros = ros

    if (!(navigator.getGamepads instanceof Function))
      console.warn('This browser does not support gamepads.')

    // setInterval(this.scanGamepads, 500)

    this.initEventListeners()
    this.update()
  }

  private initEventListeners() {
    window.addEventListener('gamepadconnected', this
      .handleConnected as EventListener)
    window.addEventListener('gamepaddisconnected', this
      .handleDisconnected as EventListener)
  }

  /**
   * TODO add support for listeners
   * @param gamepad
   */
  private handleGamepadInput(gamepad: CustomGamepad) {
    if (gamepad.getButtonPressed('a')) {
      console.log('a pressed')
    }

    const xAxis = gamepad.getAxis('left stick x')
    const yAxis = gamepad.getAxis('left stick y')
    const rightTrigger = gamepad.getButtonValue('right trigger')

    const twist = mapGamepadToTwist(xAxis, yAxis, rightTrigger)
    const topic: TopicOptions = {
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist',
    }
    this.ros.publish(topic, twist)
  }

  private update = () => {
    this.scanGamepads()
    const gamepad = this.gamepads[0]
    if (gamepad) this.handleGamepadInput(gamepad)

    requestAnimationFrame(this.update)
  }

  private scanGamepads() {
    const navigatorGamepads = [...navigator.getGamepads()]

    this.gamepads = navigatorGamepads
      .filter((g): g is Gamepad => Boolean(g))
      .map(gamepad => {
        return new CustomGamepad(gamepad)
      })
  }

  private handleConnected = (e: GamepadEvent) => {
    const gamepad = e.gamepad
    console.log(gamepad)
    this.gamepads[gamepad.index] = new CustomGamepad(gamepad)
  }

  private handleDisconnected = (e: GamepadEvent) => {
    const gamepad = e.gamepad
    delete this.gamepads[gamepad.index]
  }
}
