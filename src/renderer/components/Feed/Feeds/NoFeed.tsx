import { styled } from '@/renderer/globalStyles/styled'
import React, { FC } from 'react'

const StyledNoFeed = styled.p``

export const NoFeed: FC<{ text: string }> = ({ text }) => (
  <StyledNoFeed>{text}</StyledNoFeed>
)
