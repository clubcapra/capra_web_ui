import { combineReducers, AnyAction } from 'redux'
import { feedSlice } from '@/renderer/store/modules/feed/reducer'
import { clearStoreCache } from '@/renderer/store/localStorage'
import { FeedState } from '@/renderer/store/modules/feed/@types'
import { RosState } from '@/renderer/store/modules/ros/@types'
import { rosSlice } from '@/renderer/store/modules/ros/reducer'

const appReducer = combineReducers({
  feed: feedSlice.reducer,
  ros: rosSlice.reducer,
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

export type GlobalState = { feed: FeedState; ros: RosState }
