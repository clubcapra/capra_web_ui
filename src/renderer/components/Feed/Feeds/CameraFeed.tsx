import { TextFeed } from '@/renderer/components/Feed/Feeds/TextFeed'
import { styled } from '@/renderer/globalStyles/styled'
import { rosService } from '@/renderer/state/ros'
import { CameraType, ICameraFeed } from '@/renderer/store/modules/feed'
import { selectVideoUrl } from '@/renderer/store/modules/ros'
import { useSelector } from '@/renderer/hooks/typedUseSelector'
import { useActor } from '@xstate/react'
import * as React from 'react'
import { FC, useEffect, useRef } from 'react'
import { log } from '@/renderer/logger'

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
type CameraProp = { flipped: boolean }

const StyledVideo = styled.video<CameraProp>`
  object-fit: cover;
  max-width: 100%;
  max-height: 100%;
  height: 100%;
  overflow: hidden;
  ${({ flipped }) => (flipped ? 'transform: scaleX(-1);' : '')}
`

const StyledImg = styled.img<CameraProp>`
  object-fit: cover;
  max-width: 100%;
  max-height: 100%;
  height: 100%;
  overflow: hidden;
  ${({ flipped }) => (flipped ? 'transform: scaleX(-1);' : '')}
`

const hasGetUserMedia = () => !!navigator?.mediaDevices?.getUserMedia

const Webcam: FC<{ deviceid: string; flipped: boolean }> = ({
  deviceid,
  flipped,
}) => {
  const videoRef = useRef<HTMLVideoElement>(null)

  useEffect(() => {
    if (!hasGetUserMedia()) {
      log.error('navigator.getUserMedia is not supported')
      return
    }
    void (async () => {
      let streamStarted = false
      while (!streamStarted) {
        try {
          const stream = await navigator.mediaDevices.getUserMedia({
            video: { deviceId: deviceid },
            audio: false,
          })
          if (videoRef && videoRef.current) {
            videoRef.current.srcObject = stream
            streamStarted = true
          }
        } catch (error) {
          log.error('failed to get stream', error)
        }
      }
    })()
  }, [deviceid])

  return hasGetUserMedia() ? (
    <StyledVideo ref={videoRef} autoPlay flipped={flipped} />
  ) : (
    <TextFeed text="webcam not supported" />
  )
}

const View: FC<Props> = ({ feed }) => {
  const source = useSelector(selectVideoUrl(feed.camera))

  switch (feed.camera.type) {
    case CameraType.MJPEG:
    case CameraType.PNG:
      return (
        <StyledImg
          src={source}
          flipped={feed.camera.flipped}
          alt="camera stream"
        />
      )
    case CameraType.VP8:
      return (
        <StyledVideo
          src={source}
          flipped={feed.camera.flipped}
          autoPlay
          preload="none"
        />
      )
    case CameraType.WEBCAM:
      return (
        <Webcam deviceid={feed.camera.topic} flipped={feed.camera.flipped} />
      )
    default:
      return <TextFeed text="stream type not supported" />
  }
}

export const CameraFeed: FC<Props> = ({ feed }) => {
  const [state] = useActor(rosService)
  const connected =
    state.matches('connected') || feed.camera.type === CameraType.WEBCAM

  useEffect(() => {
    log.debug('mounting camera', feed.camera.name)
    return () => {
      log.debug('unmounting camera', feed.camera.name)
    }
  }, [feed])

  return (
    <CameraGrid>
      {connected ? <View feed={feed} /> : <TextFeed text="No video" />}
    </CameraGrid>
  )
}
