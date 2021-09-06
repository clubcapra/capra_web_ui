import {
  clearStoreCache,
  loadState,
  saveState,
} from '@/renderer/store/localStorage'
import { feedSlice } from '@/renderer/store/modules/feed/reducer'
import { rosSlice } from '@/renderer/store/modules/ros'
import { configureStore } from '@reduxjs/toolkit'
import { throttle } from 'lodash'
import { AnyAction, combineReducers } from 'redux'

const appReducer = combineReducers({
  feed: feedSlice.reducer,
  ros: rosSlice.reducer,
})

export type GlobalState = ReturnType<typeof appReducer>

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

export const store = configureStore({
  reducer: rootReducer,
  preloadedState: loadState(),
})

store.subscribe(
  throttle(() => {
    saveState(store.getState())
  }, 2000)
)
