import { Select } from '@/renderer/components/common/Select'
import { styled } from '@/renderer/globalStyles/styled'
import {
  feedSlice,
  FeedType,
  FeedTypeEnum,
  selectAllFeeds,
} from '@/renderer/store/modules/feed'
import React, { ChangeEvent, FC } from 'react'
import { useDispatch, useSelector } from 'react-redux'

interface FeedSelectProps {
  id: string
  visible: boolean
  currentFeedId: string
}

const getLabel = (feed: FeedType) => {
  switch (feed.type) {
    case FeedTypeEnum.Camera:
      return feed.camera.name
    case FeedTypeEnum.Graph:
      return feed.graph.name
    default:
      return feed.id
  }
}

export const FeedSelect: FC<FeedSelectProps> = ({
  id,
  visible,
  currentFeedId,
}) => {
  const dispatch = useDispatch()
  const allFeeds = useSelector(selectAllFeeds)
  const feedCollection = Object.values(allFeeds)

  const selectFeed = (e: ChangeEvent<HTMLSelectElement>) =>
    dispatch(feedSlice.actions.updateFeedMap({ id, feedId: e.target.value }))

  return (
    <StyledContainer hidden={!visible}>
      <Select
        onChange={selectFeed}
        value={currentFeedId}
        options={feedCollection.map((feed) => ({
          key: feed.id,
          value: feed.id,
          content: getLabel(feed),
        }))}
      />
    </StyledContainer>
  )
}

const StyledContainer = styled.div`
  position: absolute;
  bottom: 4px;
  right: 4px;
`
