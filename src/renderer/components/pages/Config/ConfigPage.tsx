import React, { FC } from 'react'
import {
  ConfigPageGrid,
  MenuArea,
  ConfigRouterArea,
} from '@/renderer/components/pages/Config/ConfigPage.styles'
import { Route, Switch, Redirect } from 'react-router'
import { ConfigMenu } from './ConfigMenu'
import { GeneralConfig } from '@/renderer/components/pages/Config/pages/GeneralConfig'
import { CameraConfig } from '@/renderer/components/pages/Config/pages/CameraConfig/CameraConfig'
import { GamepadConfig } from '@/renderer/components/pages/Config/pages/GamepadConfig/GamepadConfig'
import { useControl } from '@/renderer/utils/hooks/useControl'
import { GraphConfig } from '@/renderer/components/pages/Config/pages/GraphConfig/GraphConfig'

export const ConfigPage: FC = () => {
  useControl('nothing')
  return (
    <ConfigPageGrid>
      <MenuArea>
        <ConfigMenu />
      </MenuArea>
      <ConfigRouterArea>
        <Redirect to="/config/general" />
        <Switch>
          <Route path="/config/general" component={GeneralConfig} />
          <Route path="/config/camera" component={CameraConfig} />
          <Route path="/config/graph" component={GraphConfig} />
          <Route path="/config/gamepad" component={GamepadConfig} />
        </Switch>
      </ConfigRouterArea>
    </ConfigPageGrid>
  )
}
