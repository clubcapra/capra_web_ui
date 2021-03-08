import React, { FC, useMemo } from 'react'
import { useSelector } from '@/renderer/utils/hooks/typedUseSelector'
import { StyledFeedComponent } from '@/renderer/components/Feed/Feed.styles'
import {
  selectAllFeeds,
  selectFeedFromFeedMap,
} from '@/renderer/store/modules/feed/reducer'
import { FeedSelect } from '@/renderer/components/Feed/FeedSelect'
import { FeedView } from '@/renderer/components/Feed/FeedView'
import { useOpenClose } from '@/renderer/utils/hooks/useOpenClose'

interface FeedProps {
  id: string
  defaultFeed?: string
}

export const Feed: FC<FeedProps> = ({ id, defaultFeed }) => {
  const [isMouseOver, onOver, onLeave] = useOpenClose()

  const mappedFeed = useSelector(selectFeedFromFeedMap(id))
  const allFeeds = useSelector(selectAllFeeds)

  const feed = useMemo(() => {
    if (mappedFeed && allFeeds[mappedFeed.feedId]) {
      return allFeeds[mappedFeed.feedId]
    }
    if (defaultFeed && allFeeds[defaultFeed]) {
      return allFeeds[defaultFeed]
    }
    return Object.values(allFeeds)[0]
  }, [allFeeds, defaultFeed, mappedFeed])

  return (
    <StyledFeedComponent
      onMouseEnter={onOver}
      onMouseLeave={onLeave}
      onMouseOver={onOver}
    >
      <FeedView feed={feed} />
      <FeedSelect id={id} visible={isMouseOver} currentFeedId={feed.id} />
    </StyledFeedComponent>
  )
}
