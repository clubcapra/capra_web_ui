import { styled } from '@/renderer/globalStyles/styled'
import React, { ChangeEvent, FC } from 'react'

interface Options {
  key: string
  value: string
  content?: string
}

interface Props {
  value: string
  options: Options[]
  onChange: (e: ChangeEvent<HTMLSelectElement>) => void
  disabled?: boolean
}

const StyledSelect = styled.select`
  display: block;
  height: ${({ theme }) => theme.inputHeight};
  padding: 6px;
  box-shadow: 0 1px 2px rgba(0, 0, 0, 0.1) inset, 0 0 0 3px transparent;
  border: 1px solid black;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  color: ${({ theme }) => theme.colors.fontLight};

  option {
    background-color: ${({ theme }) => theme.colors.darkerBackground};
    color: ${({ theme }) => theme.colors.fontLight};
  }

  &:disabled {
    cursor: not-allowed;
  }
`

export const Select: FC<Props> = ({ value, options, onChange, disabled }) => {
  return (
    <StyledSelect value={value} onChange={onChange} disabled={disabled}>
      {options.map((option) => (
        <option key={option.key} value={option.value}>
          {option.content || option.value}
        </option>
      ))}
    </StyledSelect>
  )
}
