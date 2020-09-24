import React, { FC, StrictMode, useEffect } from 'react'
import { HashRouter } from 'react-router-dom'
import { ThemeProvider } from 'styled-components'
import { Provider } from 'react-redux'
import { ToastContainer } from 'react-toastify'
// import 'react-toastify/dist/ReactToastify.css'

import { defaultTheme } from '~globalStyles/themes/defaultTheme'
import { Layout } from '~components/Layout'
import { GlobalStyles } from '~App.styles'
import { store } from '~store/store'
import inputsys from '~utils/InputSystem'

const App: FC = () => {
  useEffect(() => {
    inputsys.start()
    return () => inputsys.stop()
  }, [])

  return (
    <StrictMode>
      <Provider store={store}>
        <ToastContainer position={'bottom-right'} />
        <HashRouter basename={process.env.PUBLIC_URL}>
          <ThemeProvider theme={defaultTheme}>
            <>
              <GlobalStyles />
              <Layout />
            </>
          </ThemeProvider>
        </HashRouter>
      </Provider>
    </StrictMode>
  )
}

export default App
