import { combineReducers, AnyAction } from 'redux'
import { rosSlice } from 'store/modules/ros/reducer'
import { feedSlice } from 'store/modules/feed/reducer'
import { gamepadSlice } from 'store/modules/gamepad/reducer'
import { pwaSlice } from 'store/modules/pwa/reducer'
import { clearStoreCache } from 'store/localStorage'

const appReducer = combineReducers({
  ros: rosSlice.reducer,
  feed: feedSlice.reducer,
  gamepad: gamepadSlice.reducer,
  pwa: pwaSlice.reducer,
})

export const RESET_STATE = { type: 'RESET_STATE' }

export const rootReducer = (
  state: GlobalState | undefined,
  action: AnyAction
) => {
  if (action.type === RESET_STATE.type) {
    state = undefined
    clearStoreCache()
  }

  return appReducer(state, action)
}

export type GlobalState = ReturnType<typeof appReducer>
