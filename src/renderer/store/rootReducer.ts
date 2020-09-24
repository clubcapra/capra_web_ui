import { combineReducers, AnyAction } from 'redux'
import { feedSlice } from '~store/modules/feed/reducer'
import { clearStoreCache } from '~store/localStorage'

const appReducer = combineReducers({
  feed: feedSlice.reducer,
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
