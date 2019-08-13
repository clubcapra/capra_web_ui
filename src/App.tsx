import React from 'react'
import { BrowserRouter } from 'react-router-dom'
import { ThemeProvider } from 'styled-components'
import { defaultTheme } from 'globalStyles/themes/defaultTheme'
//@ts-ignore
import { ToastProvider } from 'react-toast-notifications'
import { Layout } from 'components/Layout'
import { GlobalStyles } from 'App.styles'

const App: React.FC = () => {
  return (
    <ToastProvider placement={'bottom-right'}>
      <BrowserRouter>
        <ThemeProvider theme={defaultTheme}>
          <>
            <GlobalStyles />
            <Layout />
          </>
        </ThemeProvider>
      </BrowserRouter>
    </ToastProvider>
  )
}

export default App
