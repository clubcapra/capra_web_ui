import {
  TypedUseSelectorHook,
  useSelector as useReduxSelector,
} from 'react-redux';
import { GlobalState } from '@/renderer/store/store';

export const useSelector: TypedUseSelectorHook<GlobalState> = useReduxSelector;
