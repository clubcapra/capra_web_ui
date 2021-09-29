import { IGraphFeed } from '@/renderer/store/modules/feed'
import { useRosSubscribe } from '@/renderer/hooks/useRosSubscribe'
import {
  Chart,
  ChartData,
  ChartOptions,
  LinearScale,
  LineController,
  LineElement,
  PointElement,
  TimeScale,
} from 'chart.js'
import 'chartjs-adapter-date-fns'
import React, { FC, useCallback, useEffect, useRef, useState } from 'react'
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

function initChart(canvas: HTMLCanvasElement) {
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
  const options: ChartOptions = {
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
  }
  return new Chart(canvas, {
    type: 'line',
    data,
    options,
  })
}

export const GraphFeed: FC<Props> = ({ feed }) => {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [chart, setChart] = useState<Chart | null>(null)

  useEffect(() => {
    if (canvasRef.current && !chart) {
      const chartInstance = initChart(canvasRef.current)
      setChart(chartInstance)
    }
    return () => {
      chart?.destroy()
    }
  }, [chart])

  useRosSubscribe(
    feed.graph.topic,
    useCallback(
      (message) => {
        if (!chart || !canvasRef.current) {
          return
        }
        chart.data.datasets[0].data.push({
          x: Date.now(),
          y: parseInt(message.data, 10),
        })
        chart.update()
      },
      [chart]
    )
  )

  return <StyledCanvas ref={canvasRef} />
}

const StyledCanvas = styled.canvas`
  background-color: ${(ctx) => ctx.theme.colors.background};
`
