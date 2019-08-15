import React, { FC } from 'react'
import { ConfigPageGrid, MenuArea, ConfigRouterArea } from 'components/pages/Config/ConfigPage.styles'
import { Route, Switch } from 'react-router';
import { ConfigMenu } from './ConfigMenu';

const RosConfig = () => <div> Ros</div >
const CameraConfig = () => <div>Camera</div>
const GamepadConfig = () => <div>Gamepad</div>

export const ConfigPage: FC = () => {
  return (
    <ConfigPageGrid>
      <MenuArea>
        <ConfigMenu />
      </MenuArea>
      <ConfigRouterArea>
        <Switch>
          <Route path="/config/ros" component={RosConfig} />
          <Route path="/config/camera" component={CameraConfig} />
          <Route path="/config/gamepad" component={GamepadConfig} />
        </Switch>
      </ConfigRouterArea>
    </ConfigPageGrid>
  )
}
