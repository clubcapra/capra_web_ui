import React, { FC, useState, useCallback, useMemo } from 'react'
import { useSelector } from 'utils/hooks/typedUseSelector'
import { StyledFeedComponent } from 'components/common/Feed/Feed.styles'
import {
  selectAllFeeds,
  selectFeedFromFeedMap,
} from 'store/modules/feed/reducer'
import { FeedSelect } from 'components/common/Feed/FeedSelect'
import { FeedView } from 'components/common/Feed/FeedView'

interface FeedProps {
  id: string
  defaultFeed?: string
}

export const Feed: FC<FeedProps> = ({ id, defaultFeed }) => {
  const [hover, setHover] = useState(false)
  const toggleHover = () => setHover(!hover)

  const mappedFeed = useSelector(selectFeedFromFeedMap(id))
  const allFeeds = useSelector(selectAllFeeds)

  const feed = useMemo(() => {
    if (mappedFeed) {
      return allFeeds[mappedFeed.feedId]
    }
    if (defaultFeed && allFeeds[defaultFeed]) {
      return allFeeds[defaultFeed]
    }
    return Object.values(allFeeds)[0]
  }, [allFeeds, defaultFeed, mappedFeed])

  return (
    <StyledFeedComponent onMouseEnter={toggleHover} onMouseLeave={toggleHover}>
      <FeedView feed={feed} />
      <FeedSelect id={id} visible={!hover} currentFeedId={feed.id} />
    </StyledFeedComponent>
  )
}
