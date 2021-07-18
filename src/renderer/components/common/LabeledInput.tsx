import React, { ChangeEvent, FC } from 'react'
import { Input } from '@/renderer/components/common/Input'
import { styled } from '@/renderer/globalStyles/styled'

const StyledLabel = styled.label`
  display: block;
  padding-bottom: 2px;
  margin-top: 4px;
  margin-bottom: 2px;
`

const StyledDiv = styled.div`
  margin-bottom: 8px;
`

interface LabeledInputProps {
  value: string
  label: string
  onChange: (event: ChangeEvent<HTMLInputElement>) => void
  type?: 'text' | 'number' | 'password'
}

export const LabeledInput: FC<LabeledInputProps> = ({
  value,
  onChange,
  label,
  type,
}) => {
  return (
    <StyledDiv>
      <StyledLabel>{label}</StyledLabel>
      <Input type={type || 'text'} value={value} onChange={onChange} />
    </StyledDiv>
  )
}
