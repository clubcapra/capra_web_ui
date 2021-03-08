import React, { FC } from 'react'
import {
  MenuList,
  StyledNavLink,
} from '@/renderer/components/pages/Config/ConfigMenu.styles'

export const ConfigMenu: FC = () => {
  return (
    <aside>
      <MenuList>
        <li>
          <StyledNavLink to="/config/general" activeClassName="is-active">
            General
          </StyledNavLink>
        </li>
        <li>
          <StyledNavLink to="/config/camera" activeClassName="is-active">
            Camera
          </StyledNavLink>
        </li>
        <li>
          <StyledNavLink to="/config/gamepad" activeClassName="is-active">
            Gamepad
          </StyledNavLink>
        </li>
      </MenuList>
    </aside>
  )
}
