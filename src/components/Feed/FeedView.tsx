import React, { FC } from 'react'
import { FeedType, FeedTypeEnum } from 'store/modules/feed/@types'
import { CameraFeed } from 'components/Feed/Feeds/CameraFeed'

interface FeedViewProps {
  feed: FeedType
}

export const FeedView: FC<FeedViewProps> = ({ feed }) => {
  switch (feed.type) {
    case FeedTypeEnum.camera:
      return <CameraFeed feed={feed} />
    default:
      return null
  }
}
