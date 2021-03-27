import React, { FC } from 'react'
import { Feed } from '@/renderer/components/Feed/Feed'
import { useControl } from '@/renderer/utils/hooks/useControl'
import { styled } from '@/renderer/globalStyles/styled'

interface PipProps {
  left?: boolean
  right?: boolean
  bottom?: boolean
  top?: boolean
}

const StyledPIP = styled.div<PipProps>`
  position: absolute;
  width: 384px;
  height: 216px;
  z-index: 2;

  border: 1px solid rgba(255, 255, 255, 0.25);

  left: ${(props) => (props.left ? `1px` : undefined)};
  right: ${(props) => (props.right ? `1px` : undefined)};
  bottom: ${(props) => (props.bottom ? `1px` : undefined)};
  top: ${(props) => (props.top ? `1px` : undefined)};
`

export const Teleop: FC = () => {
  useControl('flipper')
  return (
    <>
      <Feed feed_id="teleop_main" defaultFeed="camera_3d_rgb" />
      <StyledPIP left bottom>
        <Feed feed_id="teleop_main_pip_1" defaultFeed="empty" />
      </StyledPIP>
      <StyledPIP left top>
        <Feed feed_id="teleop_main_pip_2" defaultFeed="empty" />
      </StyledPIP>
      <StyledPIP right top>
        <Feed feed_id="teleop_main_pip_3" defaultFeed="empty" />
      </StyledPIP>
    </>
  )
}
