import React, { FC } from 'react'
import { styled } from 'globalStyles/styled'

export const StyledStopButton = styled.div`
  background-color: ${({ theme }) => theme.colors.primary};
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  user-select: none;
  color: ${({ theme }) => theme.colors.fontLight};

  &:hover {
    box-shadow: inset 0 0 4px #000000;
  }

  span {
    font-weight: bold;
    font-size: 2em;
    writing-mode: vertical-rl;
    text-orientation: upright;
  }
`

export const EStopButton: FC = () => {
  const onClick = (): void => {
    console.log('EStop clicked')
  }

  return (
    <StyledStopButton onClick={onClick}>
      <span>EMERGENCY STOP</span>
    </StyledStopButton>
  )
}
