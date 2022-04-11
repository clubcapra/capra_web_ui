import React, { FC } from 'react'
import { EStopButton } from '@/renderer/components/EStopButton'
import { Header } from '@/renderer/components/Header'
import { StatusBar } from '@/renderer/components/StatusBar'
import { FlippersView } from '@/renderer/components/FlippersView'
import { Router } from '@/renderer/components/Router'
import { styled } from '@/renderer/globalStyles/styled'
import { selectFlippersViewToggleVisible } from '@/renderer/store/modules/flippersViewToggle'
import { useSelector } from 'react-redux'

const eStopButtonestopSpace = 70
const statusBarSpace = 20

const GridLayout = styled.div`
  height: 100%;
  display: grid;
  grid-template-areas:
    'h e'
    'v e'
    's e';
  grid-template-columns: 1fr ${eStopButtonestopSpace}px;
  grid-template-rows: auto 1fr ${statusBarSpace}px;
`

const StyledView = styled.div`
  grid-area: v;
  height: 100%;
  overflow-y: auto;
  position: relative;
`

const StyledFlippersViewArea = styled.div`
  grid-area: v;
  height: 75px;
  width: 300px;
  position: absolute;
  bottom: ${statusBarSpace}px;
  right: ${eStopButtonestopSpace}px;
`

const StatusBarArea = styled.div`
  grid-area: s;
`

const EStopArea = styled.div`
  grid-area: e;
`

const HeaderArea = styled.div`
  grid-area: h;
`

export const Layout: FC = () => {
  return (
    <GridLayout>
      <HeaderArea>
        <Header />
      </HeaderArea>
      <StyledView>
        <Router />
      </StyledView>
      <FlippersViewArea />
      <StatusBarArea>
        <StatusBar />
      </StatusBarArea>
      <EStopArea>
        <EStopButton />
      </EStopArea>
    </GridLayout>
  )
}

const FlippersViewArea = () => {
  return useSelector(selectFlippersViewToggleVisible) ? (
    <StyledFlippersViewArea>
      <FlippersView />
    </StyledFlippersViewArea>
  ) : null
}
