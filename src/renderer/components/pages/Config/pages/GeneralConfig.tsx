import React, { FC, ChangeEvent } from 'react'
import { LabeledInput } from '@/renderer/components/common/LabeledInput'
import { Button } from '@/renderer/components/common/Button'
import { SectionTitle } from '@/renderer/components/pages/Config/styles'
import { useActor } from '@xstate/react'
import { rosService } from '@/renderer/state/ros'
import { terminalService } from '@/renderer/state/terminal'
import { clearStoreCache } from '@/renderer/store/localStorage'
import { useDispatch } from 'react-redux'
import {
  rosSlice,
  selectBaseLinkName,
  selectDescriptionServerPort,
  selectIP,
  selectNamespace,
  selectPort,
} from '@/renderer/store/modules/ros'
import { useSelector } from '@/renderer/utils/hooks/typedUseSelector'

const ConnectionSection = () => {
  const IP = useSelector(selectIP)
  const port = useSelector(selectPort)
  const dispatch = useDispatch()
  const [state, send] = useActor(rosService)

  const updateIp = (e: ChangeEvent<HTMLInputElement>) => {
    dispatch(rosSlice.actions.updateIP(e.target.value))
  }

  const updatePort = (e: ChangeEvent<HTMLInputElement>) => {
    dispatch(rosSlice.actions.updatePort(e.target.value))
  }

  const connect = () => send('CONNECT')
  const disconnect = () => send('DISCONNECT')

  return (
    <>
      <SectionTitle>Connection</SectionTitle>
      <div style={{ display: 'flex' }}>
        <LabeledInput label="IP address" value={IP} onChange={updateIp} />
        <LabeledInput
          label="rosbrige_server port"
          value={port}
          onChange={updatePort}
        />
      </div>

      <div style={{ display: 'flex' }}>
        <Button
          onClick={connect}
          disabled={state.matches('connecting') || state.matches('connected')}
        >
          Connect
        </Button>
        <Button
          onClick={disconnect}
          disabled={
            state.matches('connecting') || state.matches('disconnected')
          }
        >
          Disconnect
        </Button>
      </div>
    </>
  )
}

const NamespaceSection = () => {
  const namespace = useSelector(selectNamespace)
  const dispatch = useDispatch()

  const updateNamespace = (e: ChangeEvent<HTMLInputElement>) =>
    dispatch(rosSlice.actions.updateNamespace(e.target.value))

  return (
    <>
      <SectionTitle>UI Ros Namespace</SectionTitle>
      <p>
        This will be appended to any topic that publishes from the UI. WARN This
        is not currently used
      </p>
      <LabeledInput
        label="Namespace"
        value={namespace}
        onChange={updateNamespace}
      />
    </>
  )
}

const UrdfDescriptionSection = () => {
  const dispatch = useDispatch()
  const descriptionServerPort = useSelector(selectDescriptionServerPort)
  const baseLinkName = useSelector(selectBaseLinkName)

  const updateDescriptionPort = (e: ChangeEvent<HTMLInputElement>): void => {
    dispatch(rosSlice.actions.updateDescriptionServerPort(e.target.value))
  }

  const updateBaseLinkName = (e: ChangeEvent<HTMLInputElement>): void => {
    dispatch(rosSlice.actions.updateBaseLinkName(e.target.value))
  }

  return (
    <>
      <SectionTitle>Urdf Description</SectionTitle>

      <LabeledInput
        label="description server port"
        value={descriptionServerPort}
        onChange={updateDescriptionPort}
      />

      <LabeledInput
        label="base_link name"
        value={baseLinkName}
        onChange={updateBaseLinkName}
      />
    </>
  )
}

const TerminalConnection = () => {
  const [state] = useActor(terminalService)

  const username = state.context.username
  const password = state.context.password

  const updateUsername = (e: ChangeEvent<HTMLInputElement>): void => {
    terminalService.send({ type: 'SET_USERNAME', username: e.target.value })
  }

  const updatePassword = (e: ChangeEvent<HTMLInputElement>): void => {
    terminalService.send({ type: 'SET_PASSWORD', password: e.target.value })
  }

  return (
    <>
      <SectionTitle>Terminal Connection Info</SectionTitle>
      <LabeledInput
        label="Username"
        value={username}
        onChange={updateUsername}
      />
      <LabeledInput
        label="Password"
        value={password}
        type="password"
        onChange={updatePassword}
      />
    </>
  )
}

const DetectedGamepad = () => {
  const gamepads = [...navigator.getGamepads()]
  return (
    <>
      <SectionTitle>Gamepads Detected</SectionTitle>
      <ul>{gamepads.map((g) => g && <li key={g?.id}>{g?.id}</li>)}</ul>
    </>
  )
}

export const GeneralConfig: FC = () => (
  <>
    <Button onClick={clearStoreCache}>Clear cache</Button>
    <ConnectionSection />
    <NamespaceSection />
    <TerminalConnection />
    <UrdfDescriptionSection />
    <DetectedGamepad />
  </>
)
