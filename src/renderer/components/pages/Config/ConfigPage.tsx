import React, { FC } from 'react'
import { Route, Routes, Navigate } from 'react-router'
import { ConfigMenu } from './ConfigMenu'
import { GeneralConfig } from '@/renderer/components/pages/Config/pages/GeneralConfig'
import { CameraConfig } from '@/renderer/components/pages/Config/pages/CameraConfig/CameraConfig'
import { GamepadConfig } from '@/renderer/components/pages/Config/pages/GamepadConfig'
import { useControl } from '@/renderer/hooks/useControl'
import { GraphConfig } from '@/renderer/components/pages/Config/pages/GraphConfig/GraphConfig'
import { styled } from '@/renderer/globalStyles/styled'
import { LaunchConfig } from './pages/LaunchConfig/LaunchConfig'
import { flippersViewToggleSlice } from '@/renderer/store/modules/flippersViewToggle'
import { useDispatch } from 'react-redux'

export const ConfigPage: FC = () => {
  useControl('nothing')

  const dispatch = useDispatch()
  dispatch(flippersViewToggleSlice.actions.setNotVisible())

  return (
    <ConfigPageGrid>
      <MenuArea>
        <ConfigMenu />
      </MenuArea>
      <ConfigRouterArea>
        <Routes>
          <Route path="/*" element={<Navigate to="/config/general" />} />
          <Route path="/general" element={<GeneralConfig />} />
          <Route path="/camera" element={<CameraConfig />} />
          <Route path="/graph" element={<GraphConfig />} />
          <Route path="/gamepad" element={<GamepadConfig />} />
          <Route path="/launch" element={<LaunchConfig />} />
        </Routes>
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
