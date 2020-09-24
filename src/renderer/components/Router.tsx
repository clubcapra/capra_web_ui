import React, { FC } from 'react'
import { Route, Switch, Redirect } from 'react-router-dom'
import { Teleop } from '~components/pages/Teleop'
import { Arm } from '~components/pages/Arm'
import { ConfigPage } from '~components/pages/Config/ConfigPage'

export const Router: FC = () => {
  return (
    <>
      <Redirect exact from="/" to="/teleop" />

      <Switch>
        <Route path="/teleop" component={Teleop} />
        <Route path="/arm" component={Arm} />
        <Route path="/config" component={ConfigPage} />
      </Switch>
    </>
  )
}
