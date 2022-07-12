import { IDetectionFeed } from '@/renderer/store/modules/feed'
import React, { FC, useCallback, useEffect, useRef, useState } from 'react'
import { styled } from '@/renderer/globalStyles/styled'
import { selectVideoUrl } from '@/renderer/store/modules/ros'
import { useSelector } from 'react-redux'
import QRScanner from 'qr-scanner'

interface Props {
  feed: IDetectionFeed
}

const StyledImg = styled.img`
  height: 100%;
  width: 100%;
  object-fit: contain;
  overflow: hidden;
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

const QRScanRegion: FC<QRScanRegionProps> = ({
  points,
  message,
  imageWidth,
  imageHeight,
}) => {
  let width = 0
  let height = 0
  let topPosition = 0
  let leftPosition = 0
  if (points.length > 0) {
    width = points[2].x - points[0].x
    height = points[2].y - points[0].y
    const yOffset = (points[0].y / imageHeight / 2) * 100
    const xOffset = (points[0].x / imageWidth / 5) * 100
    topPosition = (points[0].y / imageHeight) * 100 + yOffset
    leftPosition = (points[0].x / imageWidth) * 100 + xOffset
  }
  return points.length > 0 ? (
    <StyledContainer
      style={{ top: `${topPosition}%`, left: `${leftPosition}%` }}
    >
      <StyledRectangle
        style={{
          width: `${width}px`,
          height: `${height}px`,
        }}
      />
      <StyledText>{message}</StyledText>
    </StyledContainer>
  ) : (
    <></>
  )
}

export const QRFeed: FC<Props> = ({ feed }) => {
  const [text, setText] = useState(Array<string>())
  const [qrCodeMessage, setMessage] = useState('')
  const [qrCodeCorners, setQrCodeCorners] = useState<Point[]>([])
  const source = useSelector(selectVideoUrl(feed.camera))
  const imageRef = useRef<HTMLImageElement>(null)

  useEffect(() => {
    const lines = [...text]
    if (!lines.includes(qrCodeMessage) && qrCodeMessage.length != 0) {
      lines.push(qrCodeMessage)
      if (lines.length > 5) {
        lines.splice(0, lines.length - 5)
      }
      setText(lines)
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [qrCodeMessage])

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
    setInterval(async () => {
      await startScanRoutine()
    }, 500)

    return () => {
      clearInterval()
    }
  }, [startScanRoutine])

  return (
    <>
      <StyledImg id="image" src={source} ref={imageRef} />
      <QRScanRegion
        points={qrCodeCorners}
        message={qrCodeMessage}
        imageWidth={imageRef.current?.width ?? 0}
        imageHeight={imageRef.current?.height ?? 0}
      />
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
