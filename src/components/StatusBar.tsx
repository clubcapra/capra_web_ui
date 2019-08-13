import React, { FC } from 'react'
import { styled } from 'globalStyles/styled'

const Wrapper = styled.div`
  height: 100%;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  box-shadow: 0 -4px 2px rgba(0, 0, 0, 0.25);
`

export const StatusBar: FC = () => {
  return <Wrapper>StatusBar</Wrapper>
}
