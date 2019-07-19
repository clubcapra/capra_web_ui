import CustomGamepad from './CustomGamepad'
import { InputHandler } from './InputHandler'

export class GamepadManager {
  private inputHandler!: InputHandler
  private isRunning = false
  private prevTimestamp!: number
  private customGamepad: CustomGamepad | null = null
  private prevGamepad!: CustomGamepad
  private currentGamepadIndex: number = 0

  constructor(inputHandler: InputHandler = new InputHandler()) {
    if (!(navigator.getGamepads instanceof Function)) {
      console.warn('This browser does not support gamepads.')
      return
    }

    this.inputHandler = inputHandler

    this.initEventListeners()
  }

  get gamepad(): CustomGamepad {
    return this._gamepad
  }

  start = () => {
    if (!this.isRunning) {
      this.isRunning = true
      this.update()
    }
  }

  stop = () => {
    this.isRunning = false
  }

  private scheduleNextUpdate = () => {
    if (this.isRunning) requestAnimationFrame(tFrame => this.update(tFrame))
  }

  private update = (tFrame?: DOMHighResTimeStamp) => {
    this.updateGamepadState()
    this.scheduleNextUpdate()
  }

  private updateGamepadState = () => {
    const gamepads = navigator.getGamepads()
    for (let i = 0; i <= gamepads.length; i++) {
      const gamepad = gamepads[i]

      if (!gamepad) return

      // Don’t do anything if the current timestamp is the same as previous
      // one, which means that the state of the gamepad hasn’t changed.
      if (gamepad.timestamp && gamepad.timestamp == this.prevTimestamp) {
        return
      }

      this.prevTimestamp = gamepad.timestamp
      this.customGamepad = new CustomGamepad(gamepad)

      if (this.customGamepad.gamepad.index === this.currentGamepadIndex) {
        this.inputHandler.handleGamepadInput(
          this.customGamepad,
          this.prevGamepad
        )
        this.prevGamepad = this.customGamepad
      } else if (this.customGamepad.isSpaceMouse) {
        console.log(this.customGamepad.gamepad)
      }
    }
  }

  private initEventListeners = () => {
    window.addEventListener('gamepadconnected', this
      .onGamepadConnected as EventListener)
    window.addEventListener('gamepaddisconnected', this
      .onGamepadDisconnected as EventListener)
  }

  private onGamepadConnected = (e: GamepadEvent) => {
    const { id, ...rest } = e.gamepad
    console.log(id, 'connected', rest)
  }
  private onGamepadDisconnected = (e: GamepadEvent) => {
    const { id, ...rest } = e.gamepad
    console.log(id, 'disconnected', rest)
  }
}

export const gamepadManagerInstance = new GamepadManager()
