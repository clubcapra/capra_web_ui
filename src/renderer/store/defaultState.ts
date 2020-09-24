import { GlobalState } from '~store/rootReducer'
import { initialState as feedState } from '~store/modules/feed/initialState'

export const defaultState: GlobalState = {
  feed: feedState,
}
