import React from 'react'
import { TerminalWrapper } from '@/renderer/components/Terminal/TerminalWrapper'
import { StyledOverlay } from '@/renderer/components/Terminal/Terminal.styles'
import { useService } from '@xstate/react'
import { terminalService } from '@/renderer/state/terminal'

export const TerminalOverlay = () => {
  const [state] = useService(terminalService)
  return (
    <StyledOverlay hidden={state.matches('hidden')}>
      <TerminalWrapper hidden={state.matches('hidden')} />
    </StyledOverlay>
  )
}
