import React, { useRef, useEffect, FC, useState } from 'react'
import { FitAddon } from 'xterm-addon-fit'
import 'xterm/css/xterm.css'
import {
  TERMINAL_MAIN,
  TERMINAL_RENDERER,
  TERMINNAL_STARTED,
} from '@/shared/constants'
const { ipcRenderer } = window.require('electron')
import { useActor } from '@xstate/react'
import { rosService } from '@/renderer/state/ros'
import { terminalService } from '@/renderer/state/terminal'
import xterm from 'xterm'
import type { Terminal } from 'xterm'
import { styled } from '@/renderer/globalStyles/styled'

const StyledTerm = styled.div`
  border: 1px solid rgba(0, 255, 0, 0.3);
  height: 100%;
  width: 100&;
`

interface Props {
  hidden: boolean
}

const term: Terminal = new xterm.Terminal({
  allowTransparency: true,
  theme: { background: 'rgba(0, 0, 0, 0.6)' },
})
const fitAddon = new FitAddon()
term.loadAddon(fitAddon)

export const TerminalWrapper: FC<Props> = ({ hidden }) => {
  const terminalRef = useRef<HTMLDivElement>(null)
  const [rosState] = useActor(rosService)
  const [terminalState] = useActor(terminalService)
  const [isInit, setIsInit] = useState(false)
  const IP = rosState.context.IP

  useEffect(() => {
    if (terminalRef?.current && !isInit && !hidden) {
      term.open(terminalRef.current)
      fitAddon.fit()

      ipcRenderer.send(TERMINAL_MAIN, TERMINNAL_STARTED, {
        host: IP,
        port: 22,
        username: terminalState.context.username,
        password: terminalState.context.password,
      })

      ipcRenderer.on(TERMINAL_RENDERER, (_, data) => {
        term.write(data)
      })

      term.onKey(({ key }) => {
        const ESC = 27
        if (key.charCodeAt(0) === ESC) {
          term.blur()
          // TODO should ESC close the overlay?
        }

        ipcRenderer.send(TERMINAL_MAIN, key)
      })

      setIsInit(true)
    }
  }, [
    IP,
    hidden,
    isInit,
    terminalRef,
    terminalState.context.password,
    terminalState.context.username,
  ])

  useEffect(() => {
    if (isInit) {
      term.focus()
    }
  }, [hidden, isInit])

  return <StyledTerm ref={terminalRef} hidden={hidden} />
}
