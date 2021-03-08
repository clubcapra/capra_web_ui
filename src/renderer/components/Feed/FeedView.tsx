import React, { FC } from 'react'
import { FeedType, FeedTypeEnum } from '@/renderer/store/modules/feed/@types'
import { CameraFeed } from '@/renderer/components/Feed/Feeds/CameraFeed'
import { NoFeed } from '@/renderer/components/Feed/Feeds/NoFeed'
import { UrdfFeed } from '@/renderer/components/Feed/Feeds/UrdfFeed'

interface FeedViewProps {
  feed: FeedType
}

export const FeedView: FC<FeedViewProps> = ({ feed }) => {
  switch (feed.type) {
    case FeedTypeEnum.camera:
      return <CameraFeed feed={feed} />
    case FeedTypeEnum.urdf:
      return <UrdfFeed feed={feed} />
    default:
      return <NoFeed text="NOT SUPPORTED" />
  }
}
