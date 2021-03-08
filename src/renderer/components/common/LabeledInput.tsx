import React, { ChangeEvent, FC } from 'react'
import {
  StyledInput,
  StyledLabel,
} from '@/renderer/components/common/LabeledInput.styles'

interface LabeledInputProps {
  value: string
  label: string
  onChange: (event: ChangeEvent<HTMLInputElement>) => void
}

export const LabeledInput: FC<LabeledInputProps> = ({
  value,
  onChange,
  label,
}) => {
  return (
    <div>
      <StyledLabel>{label}</StyledLabel>
      <StyledInput type="text" value={value} onChange={onChange} />
    </div>
  )
}
