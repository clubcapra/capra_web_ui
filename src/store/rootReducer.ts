import { combineReducers } from 'redux'
import { rosSlice } from 'store/modules/ros/reducer'
import { feedSlice } from 'store/modules/feed/reducer'
import { gamepadSlice } from 'store/modules/gamepad/reducer'

export const rootReducer = combineReducers({
  ros: rosSlice.reducer,
  feed: feedSlice.reducer,
  gamepad: gamepadSlice.reducer,
})

export type GlobalState = ReturnType<typeof rootReducer>
