import { TopicOptions } from '@/utils/ros/types'
import CustomGamepad from './CustomGamepad'
import { mapGamepadToTwist } from '@/utils/math/index'
import RosClient from '@/utils/ros/RosClient'
import { Stick, GamepadBtn } from './mappings/types'

export default class GamepadManager {
  private gamepads: Array<CustomGamepad> = []
  private ros: RosClient

  count = 0

  constructor(ros: RosClient) {
    this.ros = ros

    if (!(navigator.getGamepads instanceof Function))
      console.warn('This browser does not support gamepads.')

    this.initEventListeners()
    this.update()
  }

  private initEventListeners() {
    window.addEventListener('gamepadconnected', this
      .onGamepadConnected as EventListener)
    window.addEventListener('gamepaddisconnected', this
      .onGamepadDisconnected as EventListener)
  }

  /**
   * TODO add support for listeners
   * @param gamepad
   */
  private handleGamepadInput(gamepad: CustomGamepad) {
    if (gamepad.getButtonPressed(GamepadBtn.A)) {
      console.log('a pressed')
    }

    const leftStick = gamepad.getStick(Stick.Left)
    const rt = gamepad.getButtonValue(GamepadBtn.RT)

    const twist = mapGamepadToTwist(
      leftStick.horizontal,
      leftStick.vertical,
      rt
    )

    this.count++

    if (this.count >= 5) {
      this.count = 0
      // console.log(
      //   'linear',
      //   twist.linear,
      //   'angular',
      //   twist.angular,
      //   'factor',
      //   rt
      // )
    }

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
    this.gamepads = [...navigator.getGamepads()]
      .filter((g): g is Gamepad => Boolean(g))
      .map(g => new CustomGamepad(g))
  }

  private onGamepadConnected = (e: GamepadEvent) => {
    const gamepad = e.gamepad
    this.gamepads[gamepad.index] = new CustomGamepad(gamepad)
  }

  private onGamepadDisconnected = (e: GamepadEvent) => {
    const gamepad = e.gamepad
    delete this.gamepads[gamepad.index]
  }
}
