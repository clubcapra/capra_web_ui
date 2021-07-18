import {
  TypedUseSelectorHook,
  useSelector as useReduxSelector,
} from 'react-redux'
import { GlobalState } from '@/renderer/store/rootReducer'

export const useSelector: TypedUseSelectorHook<GlobalState> = useReduxSelector
