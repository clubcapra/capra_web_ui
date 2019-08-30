import React, { ChangeEvent, useCallback, FC } from 'react'
import { useDispatch } from 'react-redux'
import { CameraType } from 'store/modules/feed/@types'
import { useSelector } from 'utils/hooks/typedUseSelector'
import {
  CameraConfigWrapper,
  CameraConfigHeaderWrapper,
  CameraAddButton as StyledAddCamera,
} from 'components/pages/Config/pages/CameraConfig/CameraConfig.styles'
import { feedSlice } from 'store/modules/feed/reducer'
import { LabeledInput } from 'components/common/LabeledInput'
import { rosSlice } from 'store/modules/ros/reducer'
import { CameraConfigTable } from './CameraConfigTable'

const VideoServerPortConfig: FC = () => {
  const dispatch = useDispatch()
  const videoServerPort = useSelector(state => state.ros.videoServerPort)

  const updateVideoServerPort = useCallback(
    (e: ChangeEvent<HTMLInputElement>) => {
      dispatch(rosSlice.actions.setVideoServerPort(e.target.value))
    },
    [dispatch]
  )

  return (
    <LabeledInput
      label="web_video_server port"
      value={videoServerPort}
      onChange={updateVideoServerPort}
    />
  )
}

const AddCamera = () => {
  const dispatch = useDispatch()

  const addFeed = useCallback(() => {
    dispatch(
      feedSlice.actions.addCamera({
        name: '',
        topic: '',
        type: CameraType.MJPEG,
      })
    )
  }, [dispatch])

  return <StyledAddCamera onClick={addFeed}>Add</StyledAddCamera>
}

export const CameraConfig: FC = () => {
  return (
    <CameraConfigWrapper>
      <CameraConfigHeaderWrapper>
        <h1>Cameras</h1>
        <AddCamera />
      </CameraConfigHeaderWrapper>
      <VideoServerPortConfig />
      <CameraConfigTable />
    </CameraConfigWrapper>
  )
}
