import React, { Suspense } from 'react'
import { useActor } from '@xstate/react'
import { terminalService } from '@/renderer/state/terminal'
import { styled } from '@/renderer/globalStyles/styled'

// For some reason if the terminal isn't loaded lazily it would load too soon
// and then try to access unintialized state
const LazyTerminalWrapper = React.lazy(() =>
  import('@/renderer/components/Terminal/TerminalWrapper').then(
    ({ TerminalWrapper }) => ({ default: TerminalWrapper })
  )
)

//WARN 70 px is the width of the estop
const StyledOverlay = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  width: calc(100% - 70px);
  height: 55%;
  z-index: 997;
`

export const TerminalOverlay = () => {
  const [state] = useActor(terminalService)

  return (
    <Suspense fallback={<div>Loading...</div>}>
      <StyledOverlay hidden={state.matches('hidden')}>
        <LazyTerminalWrapper hidden={state.matches('hidden')} />
      </StyledOverlay>
    </Suspense>
  )
}
