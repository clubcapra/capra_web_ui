import { InputHandler, GamepadData } from 'utils/gamepad/@types'
import { detectMapping, isSupported } from 'utils/gamepad/GamepadUtils'

export class GamepadManager {
  private inputHandler!: InputHandler
  private isRunning = false
  private prevTimestamp!: number[]
  private prevGamepad!: Gamepad[]
  private isBrowserSupported: boolean = true

  constructor(inputHandler: (data: GamepadData) => void) {
    if (!isSupported()) {
      console.warn('This browser does not support gamepads.')
      this.isBrowserSupported = false
      return
    }

    this.inputHandler = inputHandler
    this.initEventListeners()
  }

  start = () => {
    if (!this.isRunning && this.isBrowserSupported) {
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

      if (gamepad.timestamp && gamepad.timestamp === this.prevTimestamp[i]) {
        return
      }

      const gamepadData: GamepadData = {
        gamepad,
        mapping: detectMapping(gamepad),
        prevGamepad: this.prevGamepad[i],
      }

      this.inputHandler(gamepadData)

      this.prevTimestamp[i] = gamepad.timestamp
      this.prevGamepad[i] = gamepad
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
