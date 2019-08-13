import { configureStore } from 'redux-starter-kit'
import { rootReducer } from 'store/rootReducer'

export const store = configureStore({
  reducer: rootReducer,
})
