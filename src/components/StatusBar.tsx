import React, { FC } from 'react'
import { styled } from 'globalStyles/styled'

const Wrapper = styled.div`
  display: grid;
  align-items: center;
  height: 100%;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  color: ${({ theme }) => theme.colors.fontLight};
  box-shadow: 0 -3px 2px rgba(0, 0, 0, 0.25);
  font-size: 14px;
`

export const StatusBar: FC = () => {
  return <Wrapper>Connected to: localhost</Wrapper>
}
