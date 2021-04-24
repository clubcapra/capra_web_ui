import React, { FC, useCallback, useEffect, useRef, useState } from 'react'
import {
  Chart,
  LinearScale,
  LineController,
  LineElement,
  TimeScale,
  PointElement,
} from 'chart.js'
import { IGraphFeed } from '@/renderer/store/modules/feed/@types'
import _ from 'lodash'
import { useInterval } from '@/renderer/utils/hooks/useInterval'
import { useRosSubscribe } from '@/renderer/utils/hooks/useRosSubscribe'
import 'chartjs-adapter-date-fns'

Chart.register(
  LineController,
  LineElement,
  LinearScale,
  TimeScale,
  PointElement
)

interface Props {
  feed: IGraphFeed
}

export const GraphFeed: FC<Props> = ({ feed }) => {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [chart, setChart] = useState<Chart | null>(null)

  useEffect(() => {
    if (canvasRef.current) {
      setChart(
        new Chart(canvasRef.current, {
          type: 'line',
          data: {
            datasets: [
              {
                data: [],
                borderColor: 'rgb(255, 0, 0)',
                pointRadius: 1,
              },
            ],
          },
          options: {
            scales: {
              x: {
                type: 'time',
                ticks: {
                  maxRotation: 0,
                  color: 'white',
                  autoSkipPadding: 8,
                },
                time: {
                  displayFormats: {
                    second: 'HH:mm:ss',
                  },
                  stepSize: 1,
                  minUnit: 'second',
                },
              },
              y: {
                type: 'linear',
                ticks: {
                  color: 'white',
                },
              },
            },
            maintainAspectRatio: false,
          },
        })
      )
    }
    return () => {
      if (chart) {
        chart.destroy()
      }
      setChart(null)
    }
  }, [])

  const updateChart = useCallback(
    (data) => {
      if (chart?.data?.datasets && chart?.options?.scales?.x) {
        chart.data.datasets[0].data.push(data)
        chart.update()
      }
    },
    [chart]
  )

  useRosSubscribe(feed.graph.topic, (message) => {
    if (typeof message.data === 'string') {
      updateChart({ x: Date.now(), y: parseInt(message.data, 10) })
    }
  })

  if (process.env.NODE_ENV === 'development') {
    useInterval(() => {
      const point = { x: Date.now(), y: _.random(0, 100) }
      updateChart(point)
    }, 500)
  }

  return (
    <>
      <canvas ref={canvasRef} id="graph_canvas" />
    </>
  )
}
