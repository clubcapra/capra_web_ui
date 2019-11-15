import { configureStore } from '@reduxjs/toolkit'
import { rootReducer } from 'store/rootReducer'
import { loadState, saveState } from 'store/localStorage'
import { throttle } from 'lodash'

export const store = configureStore({
  reducer: rootReducer,
  preloadedState: loadState(),
})

store.subscribe(
  throttle(() => {
    saveState(store.getState())
  }, 2000)
)
