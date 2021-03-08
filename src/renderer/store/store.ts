import { configureStore } from '@reduxjs/toolkit'
import { rootReducer } from '@/renderer/store/rootReducer'
import { loadState, saveState } from '@/renderer/store/localStorage'
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
