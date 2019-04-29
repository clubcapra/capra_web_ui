import { GamepadMapping } from './types'

const mappings: GamepadMapping[] = [
  //xbox360
  require('./xbox360-chrome-windows-osx.json'),
  require('./xbox360-firefox-linux.json'),

  //xbone
  require('./xbone-chrome-linux.json'),
]

export default mappings
