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
import React, { FC, useEffect, useRef, useState } from 'react'
import { log } from '@/renderer/logger'
import { styled } from '@/renderer/globalStyles/styled'

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

// eslint-disable-next-line @typescript-eslint/no-explicit-any
function initChart(canvas: any) {
  log.info('Initialising chart')
  const data: ChartData = {
    datasets: [
      {
        data: [],
        borderColor: 'rgb(255, 0, 0)',
        pointRadius: 1,
      },
    ],
  }
  return new Chart(canvas, {
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
}

export const GraphFeed: FC<Props> = ({ feed }) => {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [chart, setChart] = useState<Chart | null>(null)

  useEffect(() => {
    // console.log('on mount')
    if (canvasRef.current) {
      const chartInstance = initChart(canvasRef.current)
      setChart(chartInstance)
    }
    return () => {
      // console.log('removing chart', chart)
      if (chart) {
        chart.destroy()
        setChart(null)
      }
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const updateChart = (data: any) => {
    if (chart) {
      // console.log('updating chart', data)
      chart.data.datasets[0].data.push(data)
      chart.update()
    }
  }

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
      <StyledCanvas ref={canvasRef} id="graph_canvas" />
    </>
  )
}

const StyledCanvas = styled.canvas`
  background-color: ${(ctx) => ctx.theme.colors.background};
`
