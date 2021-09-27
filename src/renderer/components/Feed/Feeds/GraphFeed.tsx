import { IGraphFeed } from '@/renderer/store/modules/feed'
import { useRosSubscribe } from '@/renderer/hooks/useRosSubscribe'
import {
  Chart,
  ChartData,
  LinearScale,
  LineController,
  LineElement,
  PointElement,
  TimeScale,
} from 'chart.js'
import 'chartjs-adapter-date-fns'
import React, { FC, useCallback, useEffect, useRef, useState } from 'react'
import { log } from '@/renderer/logger'

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
      const data: ChartData = {
        datasets: [
          {
            data: [],
            borderColor: 'rgb(255, 0, 0)',
            pointRadius: 1,
          },
        ],
      }
      setChart(
        new Chart(canvasRef.current, {
          type: 'line',
          data,
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
  }, [chart])

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
    } else {
      if (process.env.NODE_ENV === 'development') {
        log.warn(
          'graph message is not a string',
          typeof message.data,
          message.data
        )
      }
    }
  })

  return (
    <>
      <canvas ref={canvasRef} id="graph_canvas" />
    </>
  )
}
