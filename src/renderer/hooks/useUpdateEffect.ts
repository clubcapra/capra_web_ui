import { useIsMounted } from '@/renderer/hooks/useIsMounted';
import { DependencyList, EffectCallback, useEffect, useRef } from 'react';

/**
 * This is identical to useEffect except it will only run on updates and not on inital mount
 *
 * Inspired by https://stackoverflow.com/a/58217148
 * @param effect
 * @param dependencies
 */
export const useUpdateEffect = function useUpdateEffect(
  effect: EffectCallback,
  dependencies?: DependencyList
) {
  const isMounted = useIsMounted();
  const isInitialMount = useRef(true);

  useEffect(() => {
    // eslint-disable-next-line @typescript-eslint/no-empty-function
    let effectCleanup = function noop() {};
    if (isInitialMount.current) {
      isInitialMount.current = false;
    } else {
      effectCleanup = effect() || effectCleanup;
    }

    return () => {
      effectCleanup();
      // eslint-disable-next-line react-hooks/exhaustive-deps
      if (!isMounted.current) {
        isInitialMount.current = true;
      }
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, dependencies);
};
