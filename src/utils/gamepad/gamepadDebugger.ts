const haveEvents = 'ongamepadconnected' in window
const controllers: Array<Gamepad> = []

function connecthandler(e: GamepadEvent) {
  addgamepad(e.gamepad)
}

function addgamepad(gamepad: Gamepad) {
  controllers[gamepad.index] = gamepad

  const controllerDiv = document.createElement('div')
  controllerDiv.setAttribute('id', 'controller' + gamepad.index)

  const t = document.createElement('h1')
  t.appendChild(document.createTextNode('gamepad: ' + gamepad.id))
  controllerDiv.appendChild(t)

  const b = document.createElement('div')
  b.className = 'buttons'
  for (let i = 0; i < gamepad.buttons.length; i++) {
    const e = document.createElement('span')
    e.className = 'button'
    //e.id = "b" + i;
    e.innerHTML = i.toString()
    b.appendChild(e)
  }
  controllerDiv.appendChild(b)

  const a = document.createElement('div')
  a.className = 'axes'
  for (let i = 0; i < gamepad.axes.length; i++) {
    const p = document.createElement('progress')
    p.className = 'axis'
    //p.id = "a" + i;
    p.setAttribute('max', '2')
    p.setAttribute('value', '1')
    p.innerHTML = i.toString()
    a.appendChild(p)
  }
  controllerDiv.appendChild(a)

  // See https://github.com/luser/gamepadtest/blob/master/index.html
  const start = document.getElementById('start')
  if (start) {
    start.style.display = 'none'
  }

  document.body.appendChild(controllerDiv)
  requestAnimationFrame(updateStatus)
}

function disconnecthandler(e: GamepadEvent) {
  removegamepad(e.gamepad)
}

function removegamepad(gamepad: Gamepad) {
  const d = document.getElementById('controller' + gamepad.index) as Node
  document.body.removeChild(d)
  delete controllers[gamepad.index]
}

function updateStatus() {
  if (!haveEvents) scangamepads()

  let i = 0
  let j
  for (j in controllers) {
    const controller = controllers[j]
    const controllerElement = document.getElementById('controller' + j)

    if (controllerElement === null) continue

    const buttons = controllerElement.getElementsByClassName('button')

    for (i = 0; i < controller.buttons.length; i++) {
      const buttonElement = buttons[i] as HTMLElement
      const button = controller.buttons[i]
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

    const axes = controllerElement.getElementsByClassName('axis')
    for (i = 0; i < controller.axes.length; i++) {
      const a = axes[i]
      a.innerHTML = i + ': ' + controller.axes[i].toFixed(4)
      a.setAttribute('value', String(controller.axes[i] + 1))
    }
  }

  requestAnimationFrame(updateStatus)
}

function scangamepads() {
  let gamepads = navigator.getGamepads ? navigator.getGamepads() : []
  gamepads = [...gamepads]

  gamepads.forEach(gamepad => {
    if (gamepad === null) return

    if (gamepad.index in controllers) {
      controllers[gamepad.index] = gamepad
    } else {
      addgamepad(gamepad)
    }
  })
}

const init = () => {
  window.addEventListener('gamepadconnected', connecthandler as EventListener)
  window.addEventListener(
    'gamepaddisconnected',
    disconnecthandler as EventListener
  )
  if (!haveEvents) {
    setInterval(scangamepads, 500)
  }
}

export default init
