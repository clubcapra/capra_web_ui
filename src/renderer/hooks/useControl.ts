import { useEffect } from 'react';
import { useActor } from '@xstate/react';
import { controlService } from '@/renderer/state/control';

export const useControl = (control: 'arm' | 'flipper' | 'nothing') => {
  const [, send] = useActor(controlService);
  useEffect(() => {
    switch (control) {
      case 'arm':
        send('CONTROL_ARM');
        break;
      case 'flipper':
        send('CONTROL_FLIPPER');
        break;
      case 'nothing':
        send('CONTROL_NOTHING');
        break;
    }
  }, [control, send]);
};
