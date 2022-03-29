import { styled } from '@/renderer/globalStyles/styled'
import { FlippersFeed } from '@/renderer/components/Feed/Feeds/FlippersFeed'
import { selectAllFlippers } from '@/renderer/store/modules/feed'
import { useSelector } from 'react-redux'
import React, { FC } from 'react'

export const FlippersBar: FC = () => {
  const allFeeds = useSelector(selectAllFlippers)
  return (
    <StyledFlippersBarWrapper>
      <LeftStatusBar>
        <FlippersFeed feed={allFeeds[0]} />
        <FlippersFeed feed={allFeeds[1]} />
      </LeftStatusBar>
      <RightStatusBar>
        <FlippersFeed feed={allFeeds[2]} />
        <FlippersFeed feed={allFeeds[3]} />
      </RightStatusBar>
    </StyledFlippersBarWrapper>
  )
}

const StyledFlippersBarWrapper = styled.div`
  display: grid;
  grid-template: 'l r';
  grid-template-columns: 1fr 1fr;
  height: 100%;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  color: ${({ theme }) => theme.colors.fontLight};
  box-shadow: 0 -2px 2px rgba(0, 0, 0, 0.25);
  font-size: 14px;
`

const padding = 6

const LeftStatusBar = styled.div`
  grid-area: l;
  display: flex;
  align-items: center;
  > * {
    padding: 0 ${padding}px;
  }
`

const RightStatusBar = styled.div`
  grid-area: r;
  display: flex;
  justify-items: center;
  justify-content: flex-end;
  > * {
    padding: 0 ${padding}px;
  }
`
