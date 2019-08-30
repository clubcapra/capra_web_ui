import React, { FC } from 'react'
import { BrowserRouter, Route, Redirect } from 'react-router-dom'
import { ThemeProvider } from 'styled-components'
import { defaultTheme } from 'globalStyles/themes/defaultTheme'
import { ToastContainer } from 'react-toastify'
import 'react-toastify/dist/ReactToastify.css'
import { Layout } from 'components/Layout'
import { GlobalStyles } from 'App.styles'
import { GamepadManager } from 'utils/gamepad/GamepadManager'
import { DefaultInputHandler } from 'utils/gamepad/InputHandler'
import { Provider } from 'react-redux'
import { store } from 'store/store'

const gamepadManagerInstance = new GamepadManager(new DefaultInputHandler())
gamepadManagerInstance.start()

const TeleopRedirect: FC = () => <Redirect to="/teleop" />

const App: React.FC = () => (
  <Provider store={store}>
    <ToastContainer position={'bottom-right'} />
    <BrowserRouter basename={process.env.PUBLIC_URL}>
      <ThemeProvider theme={defaultTheme}>
        <>
          <GlobalStyles />
          <Route exact path="/" component={TeleopRedirect} />
          <Layout />
        </>
      </ThemeProvider>
    </BrowserRouter>
  </Provider>
)

export default App
