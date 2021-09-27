import React, { CSSProperties, FC } from 'react'
import { styled, Theme } from '@/renderer/globalStyles/styled'

type ButtonType = 'danger' | 'success' | 'primary'

interface Props {
  onClick: () => void
  btnType?: ButtonType
  disabled?: boolean
  style?: CSSProperties
}

interface StyledButtonProps {
  btnType: ButtonType | undefined
}

const getColor = (
  theme: Theme,
  type: ButtonType | undefined,
  defaultColor: string
) => {
  switch (type) {
    case 'danger':
    case 'success':
    case 'primary':
      return theme.colors[type]
    default:
      return defaultColor
  }
}

const StyledButton = styled.button<StyledButtonProps>`
  /* cursor: pointer; */
  transition: all 0.1s ease;
  user-select: none;

  display: flex;
  flex: 1 1 28px;
  /* flex-basis: 100%; */
  min-width: 0;
  background-color: transparent;
  border-color: ${({ theme, btnType }) =>
    getColor(theme, btnType, theme.colors.border)};
  border-width: 1px;
  border-style: solid;
  color: ${({ theme, btnType }) => getColor(theme, btnType, 'inherit')};
  text-align: left;
  height: 28px;
  padding: 0 6px;
  align-items: center;
  justify-content: center;
  white-space: nowrap;
  vertical-align: middle;
  background-image: linear-gradient(
    ${({ theme }) => theme.colors.background},
    ${({ theme }) => theme.colors.darkerBackground}
  );

  &:hover {
    background-image: linear-gradient(
      ${({ theme, btnType }) =>
        getColor(theme, btnType, theme.colors.darkerBackground)},
      ${({ theme, btnType }) =>
        getColor(theme, btnType, theme.colors.darkerBackground)}
    );

    color: ${({ theme }) => theme.colors.fontLight};
  }

  &:focus {
    outline: 0;
  }

  &:disabled {
    cursor: not-allowed;
  }
`

export const Button: FC<Props> = ({
  children,
  onClick,
  btnType: type,
  disabled = false,
  style,
}) => {
  return (
    <StyledButton
      onClick={onClick}
      btnType={type}
      disabled={disabled}
      style={style}
    >
      {children}
    </StyledButton>
  )
}
