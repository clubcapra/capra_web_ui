import React, { FC } from 'react'
import { Feed } from 'components/Feed/Feed'
import { controlService } from 'state/control'
import { useService } from '@xstate/react'

export const Arm: FC = () => {
  const [, send] = useService(controlService)
  send({ type: 'CONTROL_ARM' })
  return <Feed id="arm_main" defaultFeed="camera_3d_rgb" />
}
