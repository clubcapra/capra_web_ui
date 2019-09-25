import { styled } from 'globalStyles/styled'

interface ShortcutBoxProps {
  position: [number, number]
}

export const StyledShortcutBox = styled.div<ShortcutBoxProps>`
  position: absolute;
  left: ${({ position }) => position[0]}px;
  top: ${({ position }) => position[1]}px;
  padding: 8px;
  border: 1px solid ${({ theme }) => theme.colors.border};
  background-color: ${({ theme }) => theme.colors.foreground};
  z-index: 3;
  display: table;
`

export const StyledLinkBox = styled.svg`
  position: absolute;
  z-index: 2;
  left: 8px;
  top: 8px;
`
