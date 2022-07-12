import { FeedTypeEnum, IDetectionFeed } from '@/renderer/store/modules/feed'
import React, { FC, useCallback, useEffect, useRef, useState } from 'react'
import { styled } from '@/renderer/globalStyles/styled'
import { CameraFeed } from './CameraFeed'
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

const engine = QRScanner.createQrEngine()

export const QRFeed: FC<Props> = ({ feed }) => {
  const [text, setText] = useState(Array<string>())
  const [qrCodeMessage, setMessage] = useState('')
  const source = useSelector(selectVideoUrl(feed.camera))
  const imageRef = useRef<HTMLImageElement>(null)
  const canvasRef = useRef<HTMLCanvasElement>(null)

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

  useEffect(() => {
    setInterval(() => {
      console.log(scanImage())
    }, 1000)

    return () => {
      console.log('unmount')
      clearTimeout()
    }
  })

  function scanImage() {
    const result = QRScanner.scanImage(source, {
      returnDetailedScanResult: true,
      qrEngine: engine,
    })
      .then((result) => {
        return result
      })
      .catch((err: string) => {
        return err
      })
    return result
  }

  return (
    <>
      <StyledImg id="image" src={source} ref={imageRef} />

      <StyledContainer>
        <StyledRectangle />
        <StyledText>Hello {':)'}</StyledText>
      </StyledContainer>
    </>
  )
}

const StyledRectangle = styled.div`
  outline: green solid 3px;
  width: 100%;
  height: 100%;
  z-index: 1000;
`

const StyledContainer = styled.div`
  position: absolute;
  top: 30%;
  left: 30%;
  width: 30%;
  height: 50%;
  z-index: 1000;
`

const StyledText = styled.p`
  font-size: 0.8rem;
  margin-top: 0.5rem;
  color: green;
  text-align: center;
  z-index: 1001;
`
