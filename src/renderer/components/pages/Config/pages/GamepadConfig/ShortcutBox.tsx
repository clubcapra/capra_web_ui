import * as React from 'react'
import {
  StyledShortcutBox,
  StyledLinkBox,
} from '@/renderer/components/pages/Config/pages/GamepadConfig/ShortcutBox.styles'

interface Props {
  from: [number, number]
  to: [number, number]
}

export const ShortcutBox: React.FC<Props> = ({ from, to, children }) => {
  return (
    <>
      <StyledShortcutBox position={from}>{children}</StyledShortcutBox>
      <StyledLinkBox width="100%" height="100%">
        <line
          x1={from[0]}
          y1={from[1]}
          x2={to[0]}
          y2={to[1]}
          strokeWidth="1"
          stroke="white"
        />
      </StyledLinkBox>
    </>
  )
}
