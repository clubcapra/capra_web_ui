import CustomGamepad from './CustomGamepad'
import { InputHandler } from './InputHandler'

export class GamepadManager {
  private inputHandler: InputHandler
  private isRunning = false
  private prevTimestamp!: number
  private _gamepad!: CustomGamepad
  private prevGamepad!: CustomGamepad

  constructor(inputHandler: InputHandler = new InputHandler()) {
    if (!(navigator.getGamepads instanceof Function))
      console.warn('This browser does not support gamepads.')

    this.inputHandler = inputHandler

    this.initEventListeners()
  }

  get gamepad(): CustomGamepad {
    return this._gamepad
  }

  start(): void {
    if (!this.isRunning) {
      this.isRunning = true
      this.update()
    }
  }

  stop(): void {
    this.isRunning = false
  }

  private scheduleNextUpdate(): void {
    if (this.isRunning) requestAnimationFrame(tFrame => this.update(tFrame))
  }

  private update(tFrame?: DOMHighResTimeStamp): void {
    this.updateGamepadStatus()
    this.scheduleNextUpdate()
  }

  private updateGamepadStatus(): void {
    const gamepad = navigator.getGamepads && navigator.getGamepads()[0]

    if (!gamepad) return

    // Don’t do anything if the current timestamp is the same as previous
    // one, which means that the state of the gamepad hasn’t changed.
    if (gamepad.timestamp && gamepad.timestamp === this.prevTimestamp) {
      return
    }

    this.prevTimestamp = gamepad.timestamp
    this._gamepad = new CustomGamepad(gamepad)

    this.inputHandler.handleGamepadInput(this._gamepad, this.prevGamepad)

    this.prevGamepad = this._gamepad
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

export const gamepadManagerInstance = new GamepadManager()
