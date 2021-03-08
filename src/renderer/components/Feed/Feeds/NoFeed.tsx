import React, { FC } from 'react'
import { styled } from '@/renderer/globalStyles/styled'

const StyledNoFeed = styled.p``

export const NoFeed: FC<{ text: string }> = ({ text }) => (
  <StyledNoFeed>{text}</StyledNoFeed>
)
