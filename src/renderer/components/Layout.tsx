import React, { FC } from 'react'
import { EStopButton } from '@/renderer/components/EStopButton'
import { Header } from '@/renderer/components/Header'
import { StatusBar } from '@/renderer/components/StatusBar'
import { FlippersBar } from '@/renderer/components/FlippersBar'
import { Router } from '@/renderer/components/Router'
import { styled } from '@/renderer/globalStyles/styled'

const GridLayout = styled.div`
  height: 100%;
  display: grid;
  grid-template-areas:
    'h e'
    'v e'
    'f e'
    's e';
  grid-template-columns: 1fr 70px;
  grid-template-rows: auto 1fr 40px 20px;
`

const StyledView = styled.div`
  grid-area: v;
  height: 100%;
  overflow-y: auto;
  position: relative;
`

const FlippersBarArea = styled.div`
  grid-area: f;
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
      <FlippersBarArea>
        <FlippersBar />
      </FlippersBarArea>
      <StatusBarArea>
        <StatusBar />
      </StatusBarArea>
      <EStopArea>
        <EStopButton />
      </EStopArea>
    </GridLayout>
  )
}
