import { combineReducers } from 'redux'
import { rosSlice } from 'store/modules/ros/reducer'
import { feedSlice } from 'store/modules/feed/reducer'

export const rootReducer = combineReducers({
  ros: rosSlice.reducer,
  feed: feedSlice.reducer,
})

export type GlobalState = ReturnType<typeof rootReducer>
