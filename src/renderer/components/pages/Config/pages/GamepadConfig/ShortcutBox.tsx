import * as React from 'react'
import { styled } from '@/renderer/globalStyles/styled'

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

interface ShortcutBoxProps {
  position: [number, number]
}

const StyledShortcutBox = styled.div<ShortcutBoxProps>`
  position: absolute;
  left: ${({ position }) => position[0]}px;
  top: ${({ position }) => position[1]}px;
  padding: 8px;
  border: 1px solid ${({ theme }) => theme.colors.border};
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  z-index: 3;
  display: table;
`

const StyledLinkBox = styled.svg`
  position: absolute;
  z-index: 2;
  left: 8px;
  top: 8px;
`
