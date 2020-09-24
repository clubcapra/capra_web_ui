import { useEffect } from 'react'
import { useService } from '@xstate/react'
import { controlService } from '~state/control'

export const useControl = (control: 'arm' | 'flipper' | 'nothing') => {
  const [, send] = useService(controlService)
  useEffect(() => {
    switch (control) {
      case 'arm':
        send('CONTROL_ARM')
        break
      case 'flipper':
        send('CONTROL_FLIPPER')
        break
      case 'nothing':
        send('CONTROL_NOTHING')
        break
    }
  }, [control, send])
}
