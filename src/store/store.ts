import { configureStore } from 'redux-starter-kit'
import { rootReducer } from 'store/rootReducer'
import { loadState, saveState } from 'store/localStorage'
import { throttle } from 'lodash-es'

export const store = configureStore({
  reducer: rootReducer,
  preloadedState: loadState(),
})

store.subscribe(
  throttle(() => {
    saveState(store.getState())
  }, 2000)
)
