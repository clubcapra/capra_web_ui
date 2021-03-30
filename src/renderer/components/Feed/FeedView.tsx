import React, { FC } from 'react'
import { FeedType, FeedTypeEnum } from '@/renderer/store/modules/feed/@types'
import { CameraFeed } from '@/renderer/components/Feed/Feeds/CameraFeed'
import { NoFeed } from '@/renderer/components/Feed/Feeds/NoFeed'
import { UrdfFeed } from '@/renderer/components/Feed/Feeds/UrdfFeed'
import { EmptyFeed } from '@/renderer/components/Feed/Feeds/EmptyFeed'
import { GraphFeed } from '@/renderer/components/Feed/Feeds/GraphFeed'

interface FeedViewProps {
  feed: FeedType
}

export const FeedView: FC<FeedViewProps> = ({ feed }) => {
  switch (feed.type) {
    case FeedTypeEnum.Empty:
      return <EmptyFeed />
    case FeedTypeEnum.Camera:
      return <CameraFeed feed={feed} />
    case FeedTypeEnum.Urdf:
      return <UrdfFeed feed={feed} />
    case FeedTypeEnum.Graph:
      return <GraphFeed feed={feed} />
    default:
      return <NoFeed text="NOT SUPPORTED" />
  }
}
