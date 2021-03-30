import React, { FC, ChangeEvent } from 'react'
import { FeedType, FeedTypeEnum } from '@/renderer/store/modules/feed/@types'
import { useDispatch, useSelector } from 'react-redux'
import {
  selectAllFeeds,
  feedSlice,
} from '@/renderer/store/modules/feed/reducer'
import { styled } from '@/renderer/globalStyles/styled'
import { Select } from '@/renderer/components/common/Select'

const StyledContainer = styled.div`
  position: absolute;
  bottom: 4px;
  right: 4px;
`

interface FeedSelectProps {
  id: string
  visible: boolean
  currentFeedId: string
}

const getLabel = (feed: FeedType) =>
  feed.type === FeedTypeEnum.Camera ? feed.camera.name : feed.id

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
