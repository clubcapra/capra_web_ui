import React, { FC, ChangeEvent } from 'react'
import { useSelector } from 'utils/hooks/typedUseSelector'
import { useDispatch } from 'react-redux'
import { rosSlice } from 'store/modules/ros/reducer'
import { rosClient } from 'utils/ros/rosClient'
import { LabeledInput } from 'components/common/LabeledInput'
import { Button } from 'components/common/Button'
import { SectionTitle } from 'components/pages/Config/styles'
import { RESET_STATE } from 'store/rootReducer'

const ConnectionSection = () => {
  const dispatch = useDispatch()

  const IP = useSelector(state => state.ros.IP)
  const port = useSelector(state => state.ros.port)

  const updateIp = (e: ChangeEvent<HTMLInputElement>): void => {
    dispatch(rosSlice.actions.setIp(e.target.value))
  }

  const updatePort = (e: ChangeEvent<HTMLInputElement>): void => {
    dispatch(rosSlice.actions.setPort(e.target.value))
  }

  const connect = () => {
    rosClient.connect(IP, port)
    dispatch(rosSlice.actions.tryToConnect())
  }

  return (
    <>
      <SectionTitle>Connection</SectionTitle>

      <LabeledInput label="IP address" value={IP} onChange={updateIp} />
      <LabeledInput
        label="rosbrige_server port"
        value={port}
        onChange={updatePort}
      />
      <Button onClick={connect}>Connect</Button>
    </>
  )
}

const UrdfDescriptionSection = () => {
  const dispatch = useDispatch()

  const descriptionPort = useSelector(state => state.ros.descriptionServerPort)
  const baseLinkName = useSelector(state => state.ros.baseLinkName)

  const updateDescriptionPort = (e: ChangeEvent<HTMLInputElement>): void => {
    dispatch(rosSlice.actions.setDescriptionServerPort(e.target.value))
  }

  const updateBaseLinkName = (e: ChangeEvent<HTMLInputElement>): void => {
    dispatch(rosSlice.actions.setBaseLinkName(e.target.value))
  }

  return (
    <>
      <SectionTitle>Urdf Description</SectionTitle>

      <LabeledInput
        label="description server port"
        value={descriptionPort}
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

export const GeneralConfig: FC = () => (
  <>
    <ConnectionSection />
    <UrdfDescriptionSection />
    <DataSection />
    <DetectedGamepad />
  </>
)
