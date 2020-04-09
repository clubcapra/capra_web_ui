import React, { FC } from 'react'
import { Feed } from 'components/Feed/Feed'
import { controlService } from 'state/control'
import { useService } from '@xstate/react'

export const Teleop: FC = () => {
  const [, send] = useService(controlService)
  send({ type: 'CONTROL_FLIPPER' })
  return <Feed id="teleop_main" defaultFeed="camera_3d_rgb" />
}
