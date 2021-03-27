import React, { FC } from 'react'
import { Route, Switch, Redirect } from 'react-router-dom'
import { Teleop } from '@/renderer/components/pages/Teleop'
import { Victim } from '@/renderer/components/pages/Victim'
import { ConfigPage } from '@/renderer/components/pages/Config/ConfigPage'

export const Router: FC = () => {
  return (
    <>
      <Redirect exact from="/" to="/teleop" />

      <Switch>
        <Route path="/teleop" component={Teleop} />
        <Route path="/victim" component={Victim} />
        <Route path="/config" component={ConfigPage} />
      </Switch>
    </>
  )
}
