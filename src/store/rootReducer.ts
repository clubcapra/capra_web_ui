import { combineReducers } from 'redux'
import { rosSlice } from 'store/modules/ros/reducer'
import { feedSlice } from 'store/modules/feed/reducer'
import { gamepadSlice } from 'store/modules/gamepad/reducer'
import { pwaSlice } from 'store/modules/pwa/reducer'

export const rootReducer = combineReducers({
  ros: rosSlice.reducer,
  feed: feedSlice.reducer,
  gamepad: gamepadSlice.reducer,
  pwa: pwaSlice.reducer,
})

export type GlobalState = ReturnType<typeof rootReducer>
