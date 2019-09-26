import { GlobalState } from 'store/rootReducer'
import { defaultState } from 'store/defaultState'

const stateKey = 'state'

export const loadState = (): GlobalState => {
  try {
    const serializedState = localStorage.getItem(stateKey)
    if (serializedState === null) {
      return defaultState
    }
    return JSON.parse(serializedState)
  } catch (err) {
    return defaultState
  }
}

export const saveState = (state: GlobalState) => {
  try {
    const serializedState = JSON.stringify(state)
    localStorage.setItem(stateKey, serializedState)
  } catch {
    console.error('failed to persist state')
  }
}

export const clearStoreCache = () => {
  localStorage.removeItem(stateKey)
}

export default { loadState, saveState }
