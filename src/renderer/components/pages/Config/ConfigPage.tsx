import React, { FC } from 'react'
import { Route, Switch, Redirect } from 'react-router'
import { ConfigMenu } from './ConfigMenu'
import { GeneralConfig } from '@/renderer/components/pages/Config/pages/GeneralConfig'
import { CameraConfig } from '@/renderer/components/pages/Config/pages/CameraConfig/CameraConfig'
import { GamepadConfig } from '@/renderer/components/pages/Config/pages/GamepadConfig'
import { useControl } from '@/renderer/hooks/useControl'
import { GraphConfig } from '@/renderer/components/pages/Config/pages/GraphConfig/GraphConfig'
import { styled } from '@/renderer/globalStyles/styled'
import { LaunchConfig } from './pages/LaunchConfig/LaunchConfig'

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
          <Route path="/config/launch" component={LaunchConfig} />
        </Switch>
      </ConfigRouterArea>
    </ConfigPageGrid>
  )
}

const ConfigPageGrid = styled.div`
  display: grid;
  grid-template: 'm r';
  grid-template-columns: 1fr 5fr;
  height: 100%;
  max-height: 100%;
`

const MenuArea = styled.div`
  grid-area: m;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  margin: 0;
  min-height: 100%;
`

const ConfigRouterArea = styled.div`
  grid-area: r;
  padding: 16px;
  overflow: auto;
`
