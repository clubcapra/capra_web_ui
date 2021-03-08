import { GlobalState } from '@/renderer/store/rootReducer'
import { initialState as feedState } from '@/renderer/store/modules/feed/initialState'

export const defaultState: GlobalState = {
  feed: feedState,
}
