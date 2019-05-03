export default class GamepadRenderer {
  private GAMEPAD_ID = 'gamepad'

  render(gamepad: Gamepad) {
    const controllerDiv = document.createElement('div')
    controllerDiv.setAttribute('id', this.GAMEPAD_ID + gamepad.index)

    this.renderTitle(gamepad, controllerDiv)
    this.renderButtons(gamepad, controllerDiv)
    this.renderAxes(gamepad, controllerDiv)

    return controllerDiv
  }

  remove(gamepad: Gamepad) {
    const node = document.getElementById(
      this.GAMEPAD_ID + gamepad.index
    ) as Node
    document.body.removeChild(node)
  }

  update(gamepad: Gamepad, gamepadElement: HTMLElement) {
    this.updateButtons(gamepad, gamepadElement)
    this.updateAxes(gamepad, gamepadElement)
  }

  private updateButtons(gamepad: Gamepad, gamepadElement: HTMLElement) {
    const buttons = gamepadElement.getElementsByClassName('button')

    for (let i = 0; i < gamepad.buttons.length; i++) {
      const buttonElement = buttons[i] as HTMLElement
      const button = gamepad.buttons[i]
      let pressed = button.value == 1.0

      if (typeof button == 'object') {
        pressed = button.pressed
      }

      const pct = Math.round(button.value * 100) + '%'
      buttonElement.style.backgroundSize = pct + ' ' + pct

      if (pressed) {
        buttonElement.className = 'button pressed'
      } else {
        buttonElement.className = 'button'
      }
    }
  }

  private updateAxes(gamepad: Gamepad, gamepadElement: HTMLElement) {
    const axes = gamepadElement.getElementsByClassName('axis')
    for (let i = 0; i < gamepad.axes.length; i++) {
      const a = axes[i]
      a.innerHTML = i + ': ' + gamepad.axes[i].toFixed(4)
      a.setAttribute('value', String(gamepad.axes[i] + 1))
    }
  }

  private renderTitle(gamepad: Gamepad, controllerDiv: HTMLDivElement) {
    const title = document.createElement('h1')
    title.appendChild(document.createTextNode('gamepad: ' + gamepad.id))
    controllerDiv.appendChild(title)
  }

  private renderAxes(gamepad: Gamepad, controllerDiv: HTMLDivElement) {
    const axes = document.createElement('div')
    axes.className = 'axes'

    gamepad.axes.forEach((_, i) => {
      const axisProgress = document.createElement('progress')
      axisProgress.className = 'axis'
      axisProgress.setAttribute('max', '2')
      axisProgress.setAttribute('value', '1')
      axisProgress.innerHTML = i.toString()
      axes.appendChild(axisProgress)
    })

    controllerDiv.appendChild(axes)
  }

  private renderButtons(gamepad: Gamepad, controllerDiv: HTMLDivElement) {
    const buttons = document.createElement('div')
    buttons.className = 'buttons'

    gamepad.buttons.forEach((_, i) => {
      const buttonSpan = document.createElement('span')
      buttonSpan.className = 'button'
      buttonSpan.innerHTML = i.toString()
      buttons.appendChild(buttonSpan)
    })

    controllerDiv.appendChild(buttons)
  }
}
