# Example usage

```typescript
const inputsys = new InputSystem([
  {
    name: 'estop',
    bindings: [
      { type: 'keyboard', code: 'Space', onKeyDown: true },
      { type: 'gamepadBtn', button: buttons.A },
    ],
    perform: ctx => {
      console.log('performed estop')
    },
  },
])
inputsys.start()
```

## Available events

- keyboard: triggers on key down or key up, needs to be specified in the binding
- gamepad: triggers when a gamepad is updated
- gamepadAxis: triggers when a gamepad axis changes
- gamepadBtn: triggers once when button is pressed
- spacemouse: triggers when a spacemouse is changed
  - only available when a spacemouse is plugged in, otherwise it will never be triggered
