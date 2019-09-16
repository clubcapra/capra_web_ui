import React, { ChangeEvent, FC } from 'react'
import { useDispatch } from 'react-redux'
import { CameraType } from 'store/modules/feed/@types'
import { useSelector } from 'utils/hooks/typedUseSelector'
import { CameraConfigWrapper } from 'components/pages/Config/pages/CameraConfig/CameraConfig.styles'
import { feedSlice } from 'store/modules/feed/reducer'
import { LabeledInput } from 'components/common/LabeledInput'
import { rosSlice } from 'store/modules/ros/reducer'
import { Table } from './Table'
import { Button } from 'components/common/Button'
import { SectionTitle } from 'components/pages/Config/styles'

const VideoServerPortConfig: FC = () => {
  const dispatch = useDispatch()
  const videoServerPort = useSelector(state => state.ros.videoServerPort)

  const updateVideoServerPort = (e: ChangeEvent<HTMLInputElement>) =>
    dispatch(rosSlice.actions.setVideoServerPort(e.target.value))

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

export const CameraConfig: FC = () => {
  return (
    <CameraConfigWrapper>
      <VideoServerSection />
      <CameraTableSection />
    </CameraConfigWrapper>
  )
}
