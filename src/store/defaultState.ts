import { GlobalState } from 'store/rootReducer'
import { initialState as feedState } from 'store/modules/feed/initialState'
import { initialState as gamepadState } from 'store/modules/gamepad/reducer'

export const defaultState: GlobalState = {
  feed: feedState,
  gamepad: gamepadState,
}
