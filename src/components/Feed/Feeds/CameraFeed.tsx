import * as React from 'react'
import { ICameraFeed, CameraType } from 'store/modules/feed/@types'

import { useSelector } from 'utils/hooks/typedUseSelector'
import { FC, useEffect, useRef } from 'react'
import { selectVideoStreamUrl } from 'store/modules/ros/reducer'
import { styled } from 'globalStyles/styled'

interface Props {
  feed: ICameraFeed
}

const CameraGrid = styled.div`
  display: grid;
  height: 100%;
  width: 100%;
  align-items: center;
  justify-items: center;
  background-color: black;
`

const StyledVideo = styled.video`
  height: 100%;
`

const StyledImg = styled.img`
  height: 100%;
`

const StyledNoVideo = styled.p`
  display: grid;
`

const NoVideo: FC<{ text: string }> = ({ text }) => (
  <StyledNoVideo>{text}</StyledNoVideo>
)

const Webcam: FC = () => {
  const videoRef = useRef<HTMLVideoElement>(null)

  useEffect(() => {
    if (navigator && navigator.getUserMedia) {
      navigator.getUserMedia(
        { video: true, audio: false },
        mediaStream => {
          if (videoRef && videoRef.current)
            videoRef.current.srcObject = mediaStream
        },
        error => {
          console.error(error)
        }
      )
    }
  }, [])

  return videoRef.current ? (
    <StyledVideo ref={videoRef} autoPlay />
  ) : (
    <NoVideo text="webcam not supported" />
  )
}

const View: FC<Props> = ({ feed }) => {
  const source = useSelector(selectVideoStreamUrl(feed.camera))

  switch (feed.camera.type) {
    case CameraType.MJPEG:
    case CameraType.PNG:
      return <StyledImg src={source} alt="camera stream" />
    case CameraType.VP8:
      return <StyledVideo src={source} autoPlay preload="none" />
    case CameraType.WEBCAM:
      return <Webcam />
    default:
      return <NoVideo text="stream type not supported" />
  }
}

export const CameraFeed: FC<Props> = ({ feed }) => {
  const connected = useSelector(
    state => state.ros.connected || feed.camera.type === CameraType.WEBCAM
  )

  return (
    <CameraGrid>
      {connected ? <View feed={feed} /> : <NoVideo text="no video" />}
    </CameraGrid>
  )
}
