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
      <div style={{ display: 'flex' }}>
        <LabeledInput label="IP address" value={IP} onChange={updateIp} />
        <LabeledInput label="Port" value={port} onChange={updatePort} />
      </div>
      <Button onClick={connect}>Connect</Button>
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

export const RosConfig: FC = () => (
  <>
    <ConnectionSection />
    <DataSection />
  </>
)
