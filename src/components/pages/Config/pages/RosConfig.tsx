import React, { FC, ChangeEvent } from 'react'
import { useDispatch } from 'react-redux'
import { LabeledInput } from 'components/common/LabeledInput'
import { Button } from 'components/common/Button'
import { SectionTitle } from 'components/pages/Config/styles'
import { RESET_STATE } from 'store/rootReducer'
import { rosService } from 'state/ros'
import { useService } from '@xstate/react'

const ConnectionSection = () => {
  const [state, send] = useService(rosService)
  const { IP, port } = state.context

  const updateIp = (e: ChangeEvent<HTMLInputElement>): void => {
    send({ type: 'SET_IP', IP: e.target.value })
  }

  const updatePort = (e: ChangeEvent<HTMLInputElement>): void => {
    send({ type: 'SET_PORT', port: e.target.value })
  }

  const connect = () => {
    send({ type: 'CONNECT' })
  }

  return (
    <>
      <SectionTitle>Connection</SectionTitle>
      <div style={{ display: 'flex' }}>
        <LabeledInput label="IP address" value={IP} onChange={updateIp} />
        <LabeledInput label="Port" value={port} onChange={updatePort} />
      </div>
      <Button onClick={connect} disabled={state.matches('connecting')}>
        Connect
      </Button>
    </>
  )
}

const DataSection = () => {
  const dispatch = useDispatch()

  return (
    <>
      <SectionTitle>Data</SectionTitle>
      <Button onClick={() => dispatch(RESET_STATE)}>Reset Default</Button>
    </>
  )
}

const DetectedGamepad = () => {
  const gamepads = [...navigator.getGamepads()]

  return (
    <>
      <SectionTitle>Gamepads Detected</SectionTitle>
      <ul>{gamepads.map(g => g && <li key={g?.id}>{g?.id}</li>)}</ul>
    </>
  )
}

export const RosConfig: FC = () => (
  <>
    <ConnectionSection />
    <DataSection />
    <DetectedGamepad />
  </>
)
