import React, { FC, useCallback, useEffect, useRef, useState } from 'react'
import { css, styled } from '@/renderer/globalStyles/styled'
import { selectVideoUrl } from '@/renderer/store/modules/ros'
import { useSelector } from 'react-redux'
import QRScanner from 'qr-scanner'
import { ICameraFeed } from '@/renderer/store/modules/feed'
import { rosService } from '@/renderer/state/ros'
import { useActor } from '@xstate/react'
import { TextFeed } from './TextFeed'

interface Props {
  feed: ICameraFeed
}

type CameraProp = { flipped: boolean; rotated: boolean }

const CameraGrid = styled.div`
  display: grid;
  height: 100%;
  width: 100%;
  align-items: center;
  justify-items: center;
  background-color: black;
  position: absolute;
`

const autoScale = css`
  height: 100%;
  width: 100%;
  object-fit: contain;
  overflow: hidden;
`

const transform = css<CameraProp>`
  transform: ${({ flipped, rotated }) =>
    `${flipped ? 'scaleX(-1)' : ''} ${rotated ? 'rotate(180deg)' : ''}`};
`

const StyledImg = styled.img<CameraProp>`
  ${autoScale}
  ${transform}
`
interface Point {
  x: number
  y: number
}

interface QRScanRegionProps {
  points: Point[]
  message: string
  imageWidth: number
  imageHeight: number
}

// This should be a component in a seperate file and could be reused for other detection cameras
const QRScanRegion: FC<QRScanRegionProps> = ({
  points,
  message,
  imageWidth,
  imageHeight,
}) => {
  return points.length > 0 ? (
    <svg
      style={{
        width: `${imageWidth}px`,
        top: 0,
        left: 0,
        position: 'absolute',
      }}
      viewBox="0 0 640 480"
    >
      <polygon
        points={points.map((point) => `${point.x} ${point.y}`).join(' ')}
        stroke="red"
        strokeWidth="5px"
        fill="transparent"
        style={{ translate: `translateY(${points[0].y - points[2].y})` }}
      >
        <StyledText>{message}</StyledText>
      </polygon>
    </svg>
  ) : (
    <></>
  )
}

export const QRFeed: FC<Props> = ({ feed }) => {
  const [qrCodeMessage, setMessage] = useState('')
  const [qrCodeCorners, setQrCodeCorners] = useState<Point[]>([])
  const source = useSelector(selectVideoUrl(feed.camera))
  const imageRef = useRef<HTMLImageElement>(null)
  const [state] = useActor(rosService)
  const connected = state.matches('connected')

  const startScanRoutine = useCallback(() => {
    if (imageRef.current !== null) {
      return new Promise((resolve) =>
        QRScanner.scanImage(
          imageRef.current !== null ? imageRef.current : source,
          {
            returnDetailedScanResult: true,
          }
        )
          .then((result) => {
            setMessage(result.data)
            setQrCodeCorners(result.cornerPoints)
          })
          .catch(() => {
            setQrCodeCorners([])
          })
          .finally(() => {
            resolve('scanned')
          })
      )
    } else {
      return undefined
    }
  }, [source])

  useEffect(() => {
    const interval = setInterval(async () => {
      await startScanRoutine()
    }, 500)

    return () => {
      clearInterval(interval)
    }
  }, [startScanRoutine])

  return (
    <>
      {connected ? (
        <CameraGrid>
          <StyledImg
            id="image"
            src={source}
            ref={imageRef}
            rotated={feed.camera.rotated}
            flipped={feed.camera.flipped}
          />
          <QRScanRegion
            points={qrCodeCorners}
            message={qrCodeMessage}
            imageWidth={imageRef.current?.width ?? 0}
            imageHeight={imageRef.current?.height ?? 0}
          />
        </CameraGrid>
      ) : (
        <TextFeed text="No video" />
      )}
    </>
  )
}

const StyledRectangle = styled.div`
  outline: red solid 3px;
  width: 100%;
  height: 100%;
  z-index: 1000;
`

const StyledContainer = styled.div`
  position: absolute;
  z-index: 1000;
`

const StyledText = styled.p`
  font-size: 0.8rem;
  margin-top: 0.5rem;
  color: red;
  text-align: center;
  z-index: 1001;
`
