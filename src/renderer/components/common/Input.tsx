import { styled } from '@/renderer/globalStyles/styled'
import { darken } from 'polished'
import React, { ChangeEvent, FC } from 'react'

const StyledInput = styled.input`
  display: block;
  height: ${({ theme }) => theme.inputHeight};
  padding: 6px;
  box-shadow: 0 1px 2px rgba(0, 0, 0, 0.1) inset, 0 0 0 3px transparent;
  border: 1px solid black;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  color: ${({ theme }) => theme.colors.fontLight};

  &:disabled {
    background-color: ${({ theme }) => darken(0.1, theme.colors.foreground)};
    color: ${({ theme }) => darken(0.2, theme.colors.fontLight)};
    cursor: not-allowed;
  }
`

interface Props {
  type?: 'text' | 'number'
  value: string
  onChange: (e: ChangeEvent<HTMLInputElement>) => void
}

export const Input: FC<Props> = ({ type, value, onChange }) => {
  return <StyledInput type={type || 'text'} value={value} onChange={onChange} />
}
