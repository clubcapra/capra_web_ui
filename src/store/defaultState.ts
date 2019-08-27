import { GlobalState } from 'store/rootReducer'
import { initialState as rosState } from 'store/modules/ros/reducer'
import { initialState as feedState } from 'store/modules/feed/initialState'
import { initialState as pwaState } from 'store/modules/pwa/reducer'
import { initialState as gamepadState } from 'store/modules/gamepad/reducer'

export const defaultState: GlobalState = {
  ros: rosState,
  feed: feedState,
  pwa: pwaState,
  gamepad: gamepadState,
}
