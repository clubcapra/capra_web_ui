import CustomGamepad from './CustomGamepad';
import { GamepadBtn } from './mappings/types';

let joySeqId = 0

export const mapGamepadToJoy = (gamepad: Gamepad) => {
  const d = new Date()
  const seconds = Math.round(d.getTime() / 1000)
  const cgamepad  = new CustomGamepad(gamepad)

  const axs  = gamepad.axes.map(x => x  < 0.09?0.0:x)

  //Add Trigger axis at the right place in the axes array
  axs.splice(2,0,cgamepad.getButtonValue(GamepadBtn.LT))
  axs.splice(5,0,cgamepad.getButtonValue(GamepadBtn.RT))


  return {
    header: {
      seq: joySeqId++,
      stamp: {
        sec: seconds,
        nsecs: 0,
      },
      frame_id: '',
    },
    axes: axs,
    buttons: gamepad.buttons.map(x => ~~x.value) //Convert to int32 (~~)
  }
}
