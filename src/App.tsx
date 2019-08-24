import React, { FC, useEffect } from 'react'
import { BrowserRouter, Route, Redirect } from 'react-router-dom'
import { ThemeProvider } from 'styled-components'
import { defaultTheme } from 'globalStyles/themes/defaultTheme'
//@ts-ignore
import { ToastProvider } from 'react-toast-notifications'
import { Layout } from 'components/Layout'
import { GlobalStyles } from 'App.styles'
import { GamepadManager } from 'utils/gamepad/GamepadManager'
import { DefaultInputHandler } from 'utils/gamepad/InputHandler'
import { Provider } from 'react-redux'
import { store } from 'store/store'
import { PwaNotifier } from 'components/PwaNotifier'
import { rosClient } from 'utils/ros/rosClient'
import { rosSlice, fullRobotIpAddress } from 'store/modules/ros/reducer'
import { useSelector } from 'common/hooks/useTypedSelector'
import { RosNotifier } from 'components/RosNotifier'

const gamepadManagerInstance = new GamepadManager(new DefaultInputHandler())
gamepadManagerInstance.start()

const RosWrapper: FC = () => {
  const fullRobotIp = useSelector(fullRobotIpAddress)

  useEffect(() => {
    rosClient.setListeners(
      () => {
        store.dispatch(rosSlice.actions.setConnected(true))
      },
      () => {
        store.dispatch(rosSlice.actions.setConnected(false))
      },
      () => {
        store.dispatch(
          rosSlice.actions.setError(`Failed to connect to: ${fullRobotIp}`)
        )
      }
    )

    rosClient.connect(fullRobotIp)
  }, [fullRobotIp])

  return null
}

const TeleopRedirect: FC = () => <Redirect to="/teleop" />

const App: React.FC = () => (
  <Provider store={store}>
    <ToastProvider placement={'bottom-right'}>
      <RosWrapper />
      <RosNotifier />
      <PwaNotifier />
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
  </Provider>
)

export default App
