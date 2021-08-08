import React, { FC, ChangeEvent } from 'react'
import { LabeledInput } from '@/renderer/components/common/LabeledInput'
import { Button } from '@/renderer/components/common/Button'
import { SectionTitle } from '@/renderer/components/pages/Config/styles'
import { useActor } from '@xstate/react'
import { rosService } from '@/renderer/state/ros'
import { clearStoreCache } from '@/renderer/store/localStorage'
import { useDispatch } from 'react-redux'
import { rosSlice, selectNamespace } from '@/renderer/store/modules/ros/reducer'
import { useSelector } from '@/renderer/utils/hooks/typedUseSelector'

const ConnectionSection = () => {
  const [state, send] = useActor(rosService)
  const { IP, port } = state.context

  const updateIp = (e: ChangeEvent<HTMLInputElement>) => {
    send({ type: 'SET_IP', IP: e.target.value })
  }

  const updatePort = (e: ChangeEvent<HTMLInputElement>) => {
    send({ type: 'SET_PORT', port: e.target.value })
  }

  const connect = () => send('CONNECT')

  return (
    <>
      <SectionTitle>Connection</SectionTitle>
      <LabeledInput label="IP address" value={IP} onChange={updateIp} />
      <LabeledInput
        label="rosbrige_server port"
        value={port}
        onChange={updatePort}
      />
      <Button onClick={connect} disabled={state.matches('connecting')}>
        Connect
      </Button>
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
      <p>This will be appended to any topic that publishes from the UI.</p>
      <LabeledInput
        label="Namespace"
        value={namespace}
        onChange={updateNamespace}
      />
    </>
  )
}

const UrdfDescriptionSection = () => {
  const [state, send] = useActor(rosService)
  const { descriptionServerPort, baseLinkName } = state.context

  const updateDescriptionPort = (e: ChangeEvent<HTMLInputElement>): void => {
    send('SET_DESCRIPTION_SERVER_PORT', { port: e.target.value })
  }

  const updateBaseLinkName = (e: ChangeEvent<HTMLInputElement>): void => {
    send('SET_BASE_LINK_NAME', { name: e.target.value })
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
    <UrdfDescriptionSection />
    <DetectedGamepad />
  </>
)
