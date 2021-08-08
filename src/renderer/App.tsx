import React, { FC, StrictMode, useEffect } from 'react'
import { HashRouter } from 'react-router-dom'
import { ThemeProvider } from 'styled-components'
import { Provider } from 'react-redux'
import { ToastContainer } from 'react-toastify'
import 'react-toastify/dist/ReactToastify.css'

import { defaultTheme } from '@/renderer/globalStyles/themes/defaultTheme'
import { Layout } from '@/renderer/components/Layout'
import { GlobalStyles } from '@/renderer/App.styles'
import { store } from '@/renderer/store/store'
import inputsys from '@/renderer/InputSystem'

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
