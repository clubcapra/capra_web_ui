import React, { ChangeEvent, FC, useEffect, useState } from 'react'
import { useDispatch } from 'react-redux'
import { CameraType } from '@/renderer/store/modules/feed'
import { feedSlice } from '@/renderer/store/modules/feed'
import { LabeledInput } from '@/renderer/components/common/LabeledInput'
import { Table } from './Table'
import { Button } from '@/renderer/components/common/Button'
import { SectionTitle } from '@/renderer/components/pages/Config/styles'
import { styled } from '@/renderer/globalStyles/styled'
import { useSelector } from '@/renderer/hooks/typedUseSelector'
import { rosSlice, selectVideoServerPort } from '@/renderer/store/modules/ros'

const VideoServerPortConfig: FC = () => {
  const videoServerPort = useSelector(selectVideoServerPort)
  const dispatch = useDispatch()

  const updateVideoServerPort = (e: ChangeEvent<HTMLInputElement>) =>
    dispatch(rosSlice.actions.updateVideoServerPort(e.currentTarget.value))

  return (
    <>
      <h3>Notice</h3>
      <p>This is the port used to connect to ros_video_server</p>
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
    <h3>Notice</h3>
    <p>
      The webcam camera type is only for debug purposes. If you want to use it
      you need to set the topic to the device id found in the Media Device Info
      section
    </p>
    <br />
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

const CameraConfigWrapper = styled.div`
  width: 100%;
  height: 100%;
`

export const CameraConfig: FC = () => {
  return (
    <CameraConfigWrapper>
      <VideoServerSection />
      <CameraTableSection />
      <MediaDevicesInfoSection />
    </CameraConfigWrapper>
  )
}
