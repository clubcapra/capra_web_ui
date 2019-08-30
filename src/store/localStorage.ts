import { GlobalState } from 'store/rootReducer'

const stateKey = 'state'

export const loadState = () => {
  try {
    const serializedState = localStorage.getItem(stateKey)
    if (serializedState === null) {
      return undefined
    }
    return JSON.parse(serializedState)
  } catch (err) {
    return undefined
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

export const deleteLocalStorage = () => {
  localStorage.removeItem(stateKey)
}

export default { loadState, saveState }
