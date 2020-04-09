import React, { FC } from 'react'
import { Feed } from 'components/Feed/Feed'
import { controlService } from 'state/control'

export const Arm: FC = () => {
  controlService.send({ type: 'CONTROL_ARM' })
  return <Feed id="arm_main" defaultFeed="camera_3d_rgb" />
}
