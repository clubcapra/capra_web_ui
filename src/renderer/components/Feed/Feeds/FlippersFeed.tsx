import { IFlippersFeed } from '@/renderer/store/modules/feed'
import { useRosSubscribe } from '@/renderer/hooks/useRosSubscribe'
import React, { FC, useCallback, useEffect, useState } from 'react'
import { styled } from '@/renderer/globalStyles/styled'

interface Props {
  feed: IFlippersFeed
}

const StyledTextFeed = styled.div`
  width: 100%;
  height: 100%;
  text-align: center;
  display: flex;
  justify-content: center;
  align-items: center;
`

export const FlippersFeed: FC<Props> = ({ feed }) => {
  const [text, settext] = useState<string>('')

  useEffect(() => {
    if (!text) {
      settext('')
    }
    return () => {}
  }, [text])

  useRosSubscribe(
    feed.flippers.topic,
    useCallback(
      (message) => {
        settext(message.data)
      },
      [text]
    )
  )
  return (
    <StyledTextFeed>
      {feed.direction} : {text}
    </StyledTextFeed>
  )
}
