import React, { ChangeEvent, FC, useEffect, useState } from 'react'
import { useDispatch } from 'react-redux'
import { CameraType } from '@/renderer/store/modules/feed/@types'
import { CameraConfigWrapper } from '@/renderer/components/pages/Config/pages/CameraConfig/CameraConfig.styles'
import { feedSlice } from '@/renderer/store/modules/feed/reducer'
import { LabeledInput } from '@/renderer/components/common/LabeledInput'
import { Table } from './Table'
import { Button } from '@/renderer/components/common/Button'
import { SectionTitle } from '@/renderer/components/pages/Config/styles'
import { useService } from '@xstate/react'
import { rosService } from '@/renderer/state/ros'

const VideoServerPortConfig: FC = () => {
  const [state, send] = useService(rosService)
  const { videoServerPort } = state.context

  const updateVideoServerPort = (e: ChangeEvent<HTMLInputElement>) =>
    send('SET_VIDEO_SERVER_PORT', { port: e.currentTarget.value })

  return (
    <>
      <LabeledInput
        label="Port"
        value={videoServerPort}
        onChange={updateVideoServerPort}
      />
    </>
  )
}

const AddCamera = () => {
  const dispatch = useDispatch()

  const addFeed = () =>
    dispatch(
      feedSlice.actions.addCamera({
        name: '',
        topic: '',
        type: CameraType.MJPEG,
      })
    )

  return <Button onClick={addFeed}>Add New Camera</Button>
}

const VideoServerSection = () => (
  <>
    <SectionTitle>Video Server Settings</SectionTitle>
    <VideoServerPortConfig />
  </>
)

const CameraTableSection = () => (
  <>
    <SectionTitle>Cameras</SectionTitle>
    <AddCamera />
    <Table />
  </>
)

const MediaDevicesInfoSection = () => {
  const [devices, setDevices] = useState<MediaDeviceInfo[]>([])
  useEffect(() => {
    void (async () => {
      setDevices(
        (await navigator.mediaDevices.enumerateDevices()).filter(
          (d) => d.kind === 'videoinput'
        )
      )
    })()
  })

  return (
    <>
      <SectionTitle>Media Devices</SectionTitle>
      {devices.map((device) => (
        <div key={device.deviceId}>{`${device.label} ${device.deviceId}`}</div>
      ))}
    </>
  )
}

export const CameraConfig: FC = () => {
  return (
    <CameraConfigWrapper>
      <VideoServerSection />
      <CameraTableSection />
      <MediaDevicesInfoSection />
    </CameraConfigWrapper>
  )
}
