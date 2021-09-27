import { GlobalState } from '@/renderer/store/store'
import { nanoid } from 'nanoid'
import { initialState as feedState } from '@/renderer/store/modules/feed'
import { initialState as rosState } from '@/renderer/store/modules/ros'
import { initialState as inputState } from '@/renderer/store/modules/input'
import { log } from '@/renderer/logger'

export const defaultState: GlobalState = {
  feed: feedState,
  ros: rosState,
  input: inputState,
}

// WARN
// This is necessary since for some reason electron doesn't clear it's cache when installing.
// This means that if we change how the data in the state is strutcured it will fail to load properly
// The nanoid() will generate a new id everytime the UI is launched so
const stateKey = `state-${process.env.npm_package_version || nanoid()}`

export const loadState = (): GlobalState => {
  try {
    const serializedState = localStorage.getItem(stateKey)
    if (serializedState === null) {
      return defaultState
    }
    // TODO instead of versioning try to validate the data
    // if it fails validation simply return defaultState
    return JSON.parse(serializedState) as GlobalState
  } catch (err) {
    return defaultState
  }
}

export const saveState = (state: GlobalState) => {
  try {
    const serializedState = JSON.stringify(state)
    localStorage.setItem(stateKey, serializedState)
  } catch {
    log.error('failed to persist state')
  }
}

export const clearStoreCache = () => {
  localStorage.removeItem(stateKey)
}
