import { IGraphFeed } from '@/renderer/store/modules/feed'
import { useRosSubscribe } from '@/renderer/hooks/useRosSubscribe'
import React, { FC, useCallback, useEffect, useState } from 'react'
import { styled } from '@/renderer/globalStyles/styled'

interface Props {
  feed: IGraphFeed
}

export const QRFeed: FC<Props> = ({ feed }) => {
  const [text, setText] = useState(Array<string>())
  const [qrCodeMessage, setMessage] = useState('')

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
        setMessage(message.data)
      },
      // eslint-disable-next-line react-hooks/exhaustive-deps
      []
    )
  )

  return (
    <>
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
    </>
  )
}

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
