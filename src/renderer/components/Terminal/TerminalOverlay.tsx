import React, { Suspense } from 'react'
// import { TerminalWrapper } from '@/renderer/components/Terminal/TerminalWrapper'
import { StyledOverlay } from '@/renderer/components/Terminal/Terminal.styles'
import { useService } from '@xstate/react'
import { terminalService } from '@/renderer/state/terminal'

// For some reason if the terminal isn't loaded lazily it would load too soon
// and then try to access unintialized state
const LazyTerminalWrapper = React.lazy(() =>
  import('@/renderer/components/Terminal/TerminalWrapper').then(
    ({ TerminalWrapper }) => ({ default: TerminalWrapper })
  )
)

export const TerminalOverlay = () => {
  const [state] = useService(terminalService)

  return (
    <Suspense fallback={<div>Loading...</div>}>
      <StyledOverlay hidden={state.matches('hidden')}>
        <LazyTerminalWrapper hidden={state.matches('hidden')} />
      </StyledOverlay>
    </Suspense>
  )
}
