import React, { FC } from 'react'
import {
  ConfigPageGrid,
  MenuArea,
  ConfigRouterArea,
} from 'components/pages/Config/ConfigPage.styles'
import { Route, Switch, Redirect } from 'react-router'
import { ConfigMenu } from './ConfigMenu'
import { RosConfig } from 'components/pages/Config/RosConfig'

const CameraConfig: FC = () => <div>Camera</div>
const GamepadConfig: FC = () => <div>Gamepad</div>

export const ConfigPage: FC = () => {
  return (
    <ConfigPageGrid>
      <MenuArea>
        <ConfigMenu />
      </MenuArea>
      <ConfigRouterArea>
        <Redirect to="/config/ros" />
        <Switch>
          <Route path="/config/ros" component={RosConfig} />
          <Route path="/config/camera" component={CameraConfig} />
          <Route path="/config/gamepad" component={GamepadConfig} />
        </Switch>
      </ConfigRouterArea>
    </ConfigPageGrid>
  )
}
