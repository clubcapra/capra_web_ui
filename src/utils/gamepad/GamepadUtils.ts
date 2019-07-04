let joySeqId = 0

export const mapGamepadToJoy = (gamepad: Gamepad) => {
  const d = new Date()
  const seconds = Math.round(d.getTime() / 1000)

  return {
    Header: {
      seq: joySeqId++,
      stamp: {
        sec: seconds,
        nsecs: 0,
      },
      frame_id: '',
    },
    axis: gamepad.axes,
    buttons: new Int32Array(gamepad.buttons.map(x =>  x.value))
  }
}
