import React, { FC, StrictMode } from 'react'
import { BrowserRouter } from 'react-router-dom'
import { ThemeProvider } from 'styled-components'
import { defaultTheme } from 'globalStyles/themes/defaultTheme'
import { ToastContainer } from 'react-toastify'
import 'react-toastify/dist/ReactToastify.css'
import { Layout } from 'components/Layout'
import { GlobalStyles } from 'App.styles'
import { GamepadManager } from 'utils/gamepad/GamepadManager'
import { Provider } from 'react-redux'
import { store } from 'store/store'
import { handleGamepadInput } from 'utils/gamepad/InputHandler'

const gamepadManagerInstance = new GamepadManager(handleGamepadInput)
gamepadManagerInstance.start()

const App: FC = () => (
  <StrictMode>
    <Provider store={store}>
      <ToastContainer position={'bottom-right'} />
      <BrowserRouter basename={process.env.PUBLIC_URL}>
        <ThemeProvider theme={defaultTheme}>
          <>
            <GlobalStyles />
            <Layout />
          </>
        </ThemeProvider>
      </BrowserRouter>
    </Provider>
  </StrictMode>
)

export default App
