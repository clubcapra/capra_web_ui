import React, { FC } from 'react'
import { Feed } from '@/renderer/components/Feed/Feed'
import { useControl } from '@/renderer/utils/hooks/useControl'
import { styled } from '@/renderer/globalStyles/styled'

export const Victim: FC = () => {
  useControl('flipper')

  return (
    <Layout>
      <Feed feed_id="victim_feed_1" defaultFeed="camera_3d_rgb" />
      <Feed feed_id="victim_feed_2" defaultFeed="camera_3d_rgb" />
      <Feed feed_id="victim_feed_3" defaultFeed="camera_3d_rgb" />
      <Feed feed_id="victim_feed_4" defaultFeed="camera_3d_rgb" />
      <Feed feed_id="victim_feed_5" defaultFeed="camera_3d_rgb" />
      <Feed feed_id="victim_feed_6" defaultFeed="camera_3d_rgb" />
    </Layout>
  )
}

const Layout = styled.div`
  display: grid;
  height: 100%;

  /* grid-template-columns: auto auto;
  grid-template-rows: auto auto 150px;
  grid-template-areas:
    'feed1 feed2'
    'feed3 feed4'
    'feed5 feed5'; */

  grid-template-columns: auto auto 400px;
  grid-template-rows: 50% 50%;
  grid-template-areas:
    'feed1 feed2 feed5'
    'feed3 feed4 feed6';

  > #victim_feed_1 {
    grid-area: feed1;
  }
  > #victim_feed_2 {
    grid-area: feed2;
  }
  > #victim_feed_3 {
    grid-area: feed3;
  }
  > #victim_feed_4 {
    grid-area: feed4;
  }
  > #victim_feed_5 {
    grid-area: feed5;
  }
  > #victim_feed_6 {
    grid-area: feed6;
  }
`
