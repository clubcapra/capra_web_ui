import React, { FC, useCallback, ChangeEvent } from 'react'
import { useSelector } from 'utils/hooks/typedUseSelector'
import { useDispatch } from 'react-redux'
import { rosSlice } from 'store/modules/ros/reducer'
import { rosClient } from 'utils/ros/rosClient'
import { LabeledInput } from 'components/common/LabeledInput'

const IpInput: FC = () => {
  const dispatch = useDispatch()
  const IP = useSelector(state => state.ros.IP)
  const updateIp = (e: ChangeEvent<HTMLInputElement>): void => {
    dispatch(rosSlice.actions.setIp(e.currentTarget.value))
  }

  return <LabeledInput label="IP address" value={IP} onChange={updateIp} />
}

const PortInput: FC = () => {
  const dispatch = useDispatch()
  const port = useSelector(state => state.ros.port)
  const updatePort = (e: ChangeEvent<HTMLInputElement>): void => {
    dispatch(rosSlice.actions.setPort(e.currentTarget.value))
  }

  return <LabeledInput label="Port" value={port} onChange={updatePort} />
}

export const RosConfig: FC = () => {
  const dispatch = useDispatch()

  const IP = useSelector(state => state.ros.IP)
  const port = useSelector(state => state.ros.port)

  const connect = useCallback(() => {
    rosClient.connect(IP, port)
    dispatch(rosSlice.actions.tryToConnect())
  }, [IP, port, dispatch])

  return (
    <>
      <h1>Ros</h1>
      <hr />
      <IpInput />
      <PortInput />
      <button onClick={connect}>Connect</button>
    </>
  )
}
