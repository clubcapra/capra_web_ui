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

interface InputOnChange {
  onChange: (e: ChangeEvent<HTMLInputElement>) => void
}

interface Text extends InputOnChange {
  type: 'text'
  value: string
}

interface Password extends InputOnChange {
  type: 'password'
  value: string
}

interface NumberInput extends InputOnChange {
  type: 'number'
  value: string | number
}

interface Checkbox extends InputOnChange {
  type: 'checkbox'
  value: boolean
}

type InputPropsType = Checkbox | Text | Password | NumberInput

export const Input: FC<InputPropsType> = (props) => {
  if (props.type === 'checkbox') {
    return (
      <StyledInput
        type={props.type}
        checked={props.value}
        onChange={props.onChange}
      />
    )
  } else {
    return (
      <StyledInput
        type={props.type}
        value={props.value}
        onChange={props.onChange}
      />
    )
  }
}
