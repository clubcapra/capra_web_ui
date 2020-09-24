import { styled } from '~globalStyles/styled'
import { darken } from 'polished'

export const StyledStopButton = styled.div`
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  user-select: none;
  background-color: ${({ theme }) => theme.colors.primary};
  color: ${({ theme }) => theme.colors.fontLight};

  &:hover {
    box-shadow: inset 0 0 2px #000000;
  }

  &:active {
    box-shadow: inset 0 0 6px #000000;
    background-color: ${({ theme }) => darken(0.05, theme.colors.primary)};
  }

  span {
    font-weight: bold;
    font-size: 1.8em;
    writing-mode: vertical-rl;
    text-orientation: upright;
  }
`
