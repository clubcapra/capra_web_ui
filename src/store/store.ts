import { configureStore } from 'redux-starter-kit'
import { rootReducer } from 'store/rootReducer'
import { loadState, saveState } from 'store/localStorage'
import { throttle } from 'lodash'
import { defaultState } from 'store/defaultState'

const persistedState = loadState()

export const store = configureStore({
  reducer: rootReducer,
  preloadedState: persistedState || defaultState,
})

store.subscribe(
  throttle(() => {
    saveState(store.getState())
  }, 5000)
)
