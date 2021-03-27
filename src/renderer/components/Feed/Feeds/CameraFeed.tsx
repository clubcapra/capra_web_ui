import * as React from 'react'
import { ICameraFeed, CameraType } from '@/renderer/store/modules/feed/@types'

import { FC, useEffect, useRef } from 'react'
import { styled } from '@/renderer/globalStyles/styled'
import { NoFeed } from '@/renderer/components/Feed/Feeds/NoFeed'
import { useService } from '@xstate/react'
import { rosService, videoUrlSelector } from '@/renderer/state/ros'

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
  position: absolute;
`

const StyledVideo = styled.video`
  height: 100%;
  overflow: hidden;
`

const StyledImg = styled.img`
  height: 100%;
`

const hasGetUserMedia = () => !!(navigator && navigator.getUserMedia)

const Webcam: FC = () => {
  const videoRef = useRef<HTMLVideoElement>(null)

  useEffect(() => {
    if (!hasGetUserMedia()) {
      console.error('navigator.getUserMedia is not supported')
      return
    }

    navigator.getUserMedia(
      { video: true, audio: false },
      (mediaStream) => {
        if (videoRef && videoRef.current) {
          videoRef.current.srcObject = mediaStream
        }
      },
      (error) => {
        console.error('failed to get user media', error)
      }
    )
  }, [])

  return hasGetUserMedia() ? (
    <StyledVideo ref={videoRef} autoPlay />
  ) : (
    <NoFeed text="webcam not supported" />
  )
}

const View: FC<Props> = ({ feed }) => {
  const [state] = useService(rosService)
  const source = videoUrlSelector(feed.camera)(state.context)

  switch (feed.camera.type) {
    case CameraType.MJPEG:
    case CameraType.PNG:
      return <StyledImg src={source} alt="camera stream" />
    case CameraType.VP8:
      return <StyledVideo src={source} autoPlay preload="none" />
    case CameraType.WEBCAM:
      return <Webcam />
    default:
      return <NoFeed text="stream type not supported" />
  }
}

export const CameraFeed: FC<Props> = ({ feed }) => {
  const [state] = useService(rosService)
  const connected =
    state.matches('connected') || feed.camera.type === CameraType.WEBCAM

  return (
    <CameraGrid>
      {connected ? <View feed={feed} /> : <NoFeed text="no video" />}
    </CameraGrid>
  )
}
