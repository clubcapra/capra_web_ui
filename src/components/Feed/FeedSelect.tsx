import React, { FC, ChangeEvent } from 'react'
import { FeedType, FeedTypeEnum } from 'store/modules/feed/@types'
import { useDispatch, useSelector } from 'react-redux'
import { selectAllFeeds, feedSlice } from 'store/modules/feed/reducer'
import { StyledFeedSelect } from 'components/Feed/FeedSelect.styles'

interface FeedSelectProps {
  id: string
  visible: boolean
  currentFeedId: string
}

const getLabel = (feed: FeedType) =>
  feed.type === FeedTypeEnum.camera ? feed.camera.name : feed.id

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
    <StyledFeedSelect
      onChange={selectFeed}
      value={currentFeedId}
      hidden={visible}
    >
      {feedCollection.map(feed => (
        <option key={feed.id} value={feed.id}>
          {getLabel(feed)}
        </option>
      ))}
    </StyledFeedSelect>
  )
}
