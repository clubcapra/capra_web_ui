import React, { FC } from 'react'
import { styled } from '~globalStyles/styled'

const StyledNoFeed = styled.p``

export const NoFeed: FC<{ text: string }> = ({ text }) => (
  <StyledNoFeed>{text}</StyledNoFeed>
)
