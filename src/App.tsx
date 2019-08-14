import React, { FC } from 'react'
import { BrowserRouter, Route, Redirect } from 'react-router-dom'
import { ThemeProvider } from 'styled-components'
import { defaultTheme } from 'globalStyles/themes/defaultTheme'
//@ts-ignore
import { ToastProvider } from 'react-toast-notifications'
import { Layout } from 'components/Layout'
import { GlobalStyles } from 'App.styles'
import { GamepadManager } from 'utils/gamepad/GamepadManager'
import { DefaultInputHandler } from 'utils/gamepad/InputHandler'

const gamepadManagerInstance = new GamepadManager(new DefaultInputHandler())
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
