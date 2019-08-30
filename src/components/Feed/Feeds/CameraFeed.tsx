import * as React from 'react'
import { ICameraFeed, CameraType } from 'store/modules/feed/@types'

import { styled } from 'globalStyles/styled'
import { useSelector } from 'utils/hooks/typedUseSelector'
import { FC } from 'react'
import { selectVideoStreamUrl } from 'store/modules/ros/reducer'

const CameraGrid = styled.div`
  display: grid;
  > * {
    display: grid;
    align-items: center;
    justify-items: center;
    height: 100%;
    width: 100%;
  }
`

interface Props {
  feed: ICameraFeed
}

const NoVideo: FC<{ text: string }> = ({ text }) => (
  <div>
    <p>{text}</p>
  </div>
)

const View: FC<Props> = ({ feed }) => {
  const source = useSelector(selectVideoStreamUrl(feed.camera))

  switch (feed.camera.type) {
    case CameraType.MJPEG:
    case CameraType.PNG:
      return <img src={source} alt="camera stream" />
    case CameraType.VP8:
      return <video src={source} autoPlay preload="none" />
    default:
      return <NoVideo text="stream type not supported" />
  }
}

export const CameraFeed: FC<Props> = ({ feed }) => {
  const connected = useSelector(state => state.ros.connected)

  return (
    <CameraGrid>
      {connected ? <View feed={feed} /> : <NoVideo text="no video" />}
    </CameraGrid>
  )
}
