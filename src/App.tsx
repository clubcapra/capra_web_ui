import React, { FC } from 'react'
import { BrowserRouter, Route, Redirect } from 'react-router-dom'
import { ThemeProvider } from 'styled-components'
import { defaultTheme } from 'globalStyles/themes/defaultTheme'
//@ts-ignore
import { ToastProvider } from 'react-toast-notifications'
import { Layout } from 'components/Layout'
import { GlobalStyles } from 'App.styles'
import { gamepadManagerInstance } from 'utils/gamepad/GamepadManager'

gamepadManagerInstance.start()

const TeleopRedirect: FC = () => <Redirect to="/teleop" />

const App: React.FC = () => {
  return (
    <ToastProvider placement={'bottom-right'}>
      <BrowserRouter>
        <ThemeProvider theme={defaultTheme}>
          <>
            <GlobalStyles />
            <Route exact path="/" component={TeleopRedirect} />
            <Layout />
          </>
        </ThemeProvider>
      </BrowserRouter>
    </ToastProvider>
  )
}

export default App