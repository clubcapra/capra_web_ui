import React, { FC } from 'react'
import { Feed } from 'components/Feed/Feed'
import { useArmControl } from 'utils/hooks/useArmControl'

export const Arm: FC = () => {
  useArmControl()
  return <Feed id="arm_main" defaultFeed="camera_3d_rgb" />
}
