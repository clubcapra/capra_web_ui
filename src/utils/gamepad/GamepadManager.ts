import CustomGamepad from './CustomGamepad'
import { InputHandler } from './InputHandler'

export default class GamepadManager {
  private gamepads: Array<CustomGamepad> = []
  private inputHandler = new InputHandler()
  private isPolling = false
  private prevTimestamp!: number

  constructor() {
    if (!(navigator.getGamepads instanceof Function))
      console.warn('This browser does not support gamepads.')

    this.initEventListeners()
    this.startPolling()
  }

  get gamepad() {
    return this.gamepads[0]
  }

  startPolling() {
    if (!this.isPolling) {
      this.isPolling = true
      this.update()
    }
  }

  stopPolling() {
    this.isPolling = false
  }

  private scheduleNextUpdate() {
    if (this.isPolling) requestAnimationFrame(tFrame => this.update(tFrame))
  }

  private update(tFrame?: DOMHighResTimeStamp) {
    this.pollStatus()
    this.scheduleNextUpdate()
  }

  private pollStatus() {
    const gamepad = navigator.getGamepads && navigator.getGamepads()[0]

    if (!gamepad) return

    // Don’t do anything if the current timestamp is the same as previous
    // one, which means that the state of the gamepad hasn’t changed.
    if (gamepad.timestamp && gamepad.timestamp == this.prevTimestamp) {
      return
    }

    this.prevTimestamp = gamepad.timestamp

    this.inputHandler.handleGamepadInput(new CustomGamepad(gamepad))
  }

  private initEventListeners = () => {
    window.addEventListener('gamepadconnected', this
      .onGamepadConnected as EventListener)
    window.addEventListener('gamepaddisconnected', this
      .onGamepadDisconnected as EventListener)
  }

  private onGamepadConnected = (e: GamepadEvent) => {}
  private onGamepadDisconnected = (e: GamepadEvent) => {}
}
