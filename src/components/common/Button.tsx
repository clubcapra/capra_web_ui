import React, { FC } from 'react'
import { styled } from 'globalStyles/styled'

interface Props {
  onClick: () => void
}

const StyledButton = styled.button`
  padding: 4px 8px;
  background-color: ${({ theme }) => theme.colors.primary};
  color: ${({ theme }) => theme.colors.fontLight};
  cursor: pointer;
  transition: all 0.1s ease;
  user-select: none;

  &:hover {
    background-color: ${({ theme }) => theme.colors.accent};
    color: ${({ theme }) => theme.colors.fontDark};
  }

  &:active {
    transition: none;
    background-color: ${({ theme }) => theme.colors.primary};
    color: ${({ theme }) => theme.colors.fontLight};
  }
`

export const Button: FC<Props> = ({ children, onClick }) => {
  return <StyledButton onClick={onClick}>{children}</StyledButton>
}
