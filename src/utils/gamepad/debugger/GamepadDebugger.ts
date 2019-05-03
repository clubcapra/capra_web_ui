import GamepadRenderer from './GamepadRenderer'

export default class GamepadDebugger {
  gamepads: Array<Gamepad | null> = []
  renderer: GamepadRenderer = new GamepadRenderer()

  constructor() {
    window.addEventListener('gamepadconnected', this
      .onGamepadConnected as EventListener)

    window.addEventListener('gamepaddisconnected', this
      .onGamepadDisconnected as EventListener)

    setInterval(this.scangamepads, 500)
  }

  onGamepadConnected(e: GamepadEvent) {
    console.log('gamepad connected', this)
    this.addGamepad(e.gamepad)
  }

  onGamepadDisconnected(e: GamepadEvent) {
    console.log('gamepad disconnected')
    this.removeGamepad(e.gamepad)
  }

  addGamepad(gamepad: Gamepad) {
    this.gamepads[gamepad.index] = gamepad

    const start = document.getElementById('start')
    if (start) {
      start.style.display = 'none'
    }

    this.renderGamepad(gamepad)

    requestAnimationFrame(this.updateStatus)
  }

  removeGamepad(gamepad: Gamepad) {
    this.renderer.remove(gamepad)
    delete this.gamepads[gamepad.index]
  }

  private renderGamepad(gamepad: Gamepad) {
    const controllerDiv = this.renderer.render(gamepad)
    document.body.appendChild(controllerDiv)
  }

  private updateStatus() {
    this.scangamepads()
    this.gamepads.forEach((gamepad, i) => {
      const gamepadElement = document.getElementById('controller' + i)

      if (gamepadElement === null) return
      if (gamepad) this.renderer.update(gamepad, gamepadElement)
    })

    requestAnimationFrame(this.updateStatus)
  }

  private scangamepads() {
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : []
    this.gamepads = [...gamepads]

    this.gamepads.forEach(gamepad => {
      if (gamepad === null) return

      if (gamepad.index in this.gamepads) {
        this.gamepads[gamepad.index] = gamepad
      } else {
        this.addGamepad(gamepad)
      }
    })
  }
}
