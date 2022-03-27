import { GraphType, IGraphFeed } from '@/renderer/store/modules/feed'
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
  const [text, setText] = useState(Array<string>())
  const [qrCodeMessage, setMessage] = useState('')

  useEffect(() => {
    if (canvasRef.current && !chart && feed.graph.type != GraphType.TEXT) {
      const chartInstance = initChart(canvasRef.current)
      setChart(chartInstance)
    }
    return () => {
      chart?.destroy()
    }
  }, [chart, feed.graph.type])

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

  useRosSubscribe(
    feed.graph.topic,
    useCallback(
      (message) => {
        switch (feed.graph.type) {
          case GraphType.TEXT:
            setMessage(message.data)
            break
          case GraphType.GRAPH:
          default:
            if (!chart || !canvasRef.current) {
              return
            }
            chart.data.datasets[0].data.push({
              x: Date.now(),
              y: parseInt(message.data, 10),
            })
            chart.update()
        }
      },
      // eslint-disable-next-line react-hooks/exhaustive-deps
      [chart]
    )
  )

  return (
    <>
      {feed.graph.type == GraphType.TEXT ? (
        <StyledTable>
          <thead>
            <tr>
              <th>Type</th>
              <th>Text</th>
            </tr>
          </thead>
          <tbody>
            {text.map((line) => {
              return (
                <tr key={line}>
                  <td>QR Code</td>
                  <td>{line}</td>
                </tr>
              )
            })}
          </tbody>
        </StyledTable>
      ) : (
        <StyledCanvas ref={canvasRef} />
      )}
    </>
  )
}

const StyledCanvas = styled.canvas`
  background-color: ${(ctx) => ctx.theme.colors.background};
`

const StyledTable = styled.table`
  width: 100%;
  border-spacing: 0;
  border-collapse: collapse;

  th,
  td {
    text-align: left;

    &:last-child {
      width: 80%;
    }
  }

  td {
    padding: 8px 8px;

    input {
      width: 100%;
    }

    select {
      width: 100%;
    }
  }

  thead th {
    padding: 4px 8px;
    font-size: inherit;
    border-bottom: 1px solid ${({ theme }) => theme.colors.border};
  }

  tbody {
    border: 1px solid ${({ theme }) => theme.colors.border};
  }
`
