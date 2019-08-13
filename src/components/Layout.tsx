import React, { FC } from 'react'
import {
  GridLayout,
  HeaderArea,
  StyledView,
  StatusBarArea,
  EStopArea,
} from './Layout.styles'
import { EStopButton } from 'components/EStopButton'
import { Header } from 'components/Header'
import { StatusBar } from 'components/StatusBar'

export const Layout: FC = ({ children }) => {
  return (
    <GridLayout>
      <HeaderArea>
        <Header />
      </HeaderArea>
      <StyledView>{children}</StyledView>
      <StatusBarArea>
        <StatusBar />
      </StatusBarArea>
      <EStopArea>
        <EStopButton />
      </EStopArea>
    </GridLayout>
  )
}
