import React, { FC } from 'react'
import { Point } from './QRFeed'

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
        points={points
          .map((point) => `${imageWidth - point.x} ${point.y}`)
          .join(' ')}
        stroke="green"
        strokeWidth="5px"
        fill="transparent"
        style={{ translate: `translateY(${points[0].y - points[2].y})` }}
      />
      <text
        x={(points[2].x - points[0].x) / 2 + imageWidth - points[2].x}
        y={points[2].y + 20}
        fill="green"
        textAnchor="middle"
      >
        {message}
      </text>
    </svg>
  ) : (
    <></>
  )
}

export default QRScanRegion
