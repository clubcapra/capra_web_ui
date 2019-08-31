import React, { FC, useCallback, ChangeEvent } from 'react'
import { useSelector } from 'utils/hooks/typedUseSelector'
import { useDispatch } from 'react-redux'
import { rosSlice } from 'store/modules/ros/reducer'
import { rosClient } from 'utils/ros/rosClient'
import { LabeledInput } from 'components/common/LabeledInput'
import { deleteLocalStorage } from 'store/localStorage'
import { Button } from 'components/common/Button'
import { SectionTitle } from 'components/pages/Config/styles'

const ConnectionSection = () => {
  const dispatch = useDispatch()

  const IP = useSelector(state => state.ros.IP)
  const port = useSelector(state => state.ros.port)

  const updateIp = useCallback(
    (e: ChangeEvent<HTMLInputElement>): void => {
      dispatch(rosSlice.actions.setIp(e.target.value))
    },
    [dispatch]
  )

  const updatePort = useCallback(
    (e: ChangeEvent<HTMLInputElement>): void => {
      dispatch(rosSlice.actions.setPort(e.target.value))
    },
    [dispatch]
  )

  const connect = useCallback(() => {
    rosClient.connect(IP, port)
    dispatch(rosSlice.actions.tryToConnect())
  }, [IP, port, dispatch])

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

const DataSection = () => (
  <>
    <SectionTitle>Data</SectionTitle>
    <Button onClick={deleteLocalStorage}>Clear Cache</Button>
  </>
)

export const RosConfig: FC = () => (
  <>
    <ConnectionSection />
    <DataSection />
  </>
)
