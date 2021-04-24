import { GlobalState } from '@/renderer/store/rootReducer'
import { initialState as feedState } from '@/renderer/store/modules/feed/initialState'
import { initialState as rosState } from '@/renderer/store/modules/ros/initialState'

export const defaultState: GlobalState = {
  feed: feedState,
  ros: rosState,
}
