import React, { FC, StrictMode, useEffect } from 'react'
import { HashRouter } from 'react-router-dom'
import { ThemeProvider } from 'styled-components'
import { Provider } from 'react-redux'
import { ToastContainer } from 'react-toastify'
import 'react-toastify/dist/ReactToastify.css'
import { TerminalOverlay } from '@/renderer/components/Terminal/TerminalOverlay'
import { defaultTheme } from '@/renderer/globalStyles/themes/defaultTheme'
import { Layout } from '@/renderer/components/Layout'
import { store } from '@/renderer/store/store'
import inputSystem from '@/renderer/inputSystem'
import { createGlobalStyle } from 'styled-components'
import { Theme } from '@/renderer/globalStyles/styled'

export const App: FC = () => {
  useEffect(() => {
    inputSystem.start()
    return () => inputSystem.stop()
  }, [])

  return (
    <StrictMode>
      <Provider store={store}>
        <ToastContainer position={'bottom-right'} theme={'dark'} icon={false} />
        <HashRouter basename={process.env.PUBLIC_URL}>
          <ThemeProvider theme={defaultTheme}>
            <>
              <GlobalStyles />
              <Layout />
              <TerminalOverlay />
            </>
          </ThemeProvider>
        </HashRouter>
      </Provider>
    </StrictMode>
  )
}

const GlobalStyles = createGlobalStyle<{ theme: Theme }>`
  html { height: 100%; }

  body {
    background: ${(props) => props.theme.colors.background};
    color: ${(props) => props.theme.colors.fontLight};
    min-height: 100%;
  }

  * {
    box-sizing: border-box;
    padding: 0;
    margin: 0;
  }

  #root {
    height: 100vh;
    --toastify-text-color-dark: ${(props) => props.theme.colors.fontLight};
    --toastify-color-dark: ${(props) => props.theme.colors.background};
    --toastify-color-success: ${(props) => props.theme.colors.success};
    --toastify-color-warning: ${(props) => props.theme.colors.warning};
    --toastify-color-error: ${(props) => props.theme.colors.danger};
  }
`
