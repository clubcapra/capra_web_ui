import React, { FC } from 'react'
import { EStopButton } from '~components/EStopButton'
import { Header } from '~components/Header'
import { StatusBar } from '~components/StatusBar/StatusBar'
import { Router } from '~components/Router'
import { styled } from '~globalStyles/styled'

const GridLayout = styled.div`
  height: 100%;
  display: grid;
  grid-template-areas:
    'h e'
    'v e'
    's e';
  grid-template-columns: 1fr 70px;
  grid-template-rows: auto 1fr 20px;
`

const StyledView = styled.div`
  grid-area: v;
  height: 100%;
  overflow-y: auto;
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
      <StatusBarArea>
        <StatusBar />
      </StatusBarArea>
      <EStopArea>
        <EStopButton />
      </EStopArea>
    </GridLayout>
  )
}
