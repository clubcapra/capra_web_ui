import React, { FC } from 'react'
import {
  HeaderGrid,
  LeftHeader,
  StyledNavLink,
  RightHeader,
  StyledLogo,
} from '@/renderer/components/Header.styles'
// import { getAssetURL } from 'electron-snowpack'

interface NavLinkDefinition {
  to: string
  label: string
}

const navLinks: NavLinkDefinition[] = [
  {
    to: '/teleop',
    label: 'Teleop',
  },
  {
    to: '/victim',
    label: 'Victim',
  },
  {
    to: '/config',
    label: 'Config',
  },
]

export const Header: FC = () => {
  return (
    <HeaderGrid>
      <LeftHeader navlinks={navLinks}>
        {navLinks.map(({ to, label }) => (
          <StyledNavLink key={to} to={to} activeClassName="is-active">
            {label}
          </StyledNavLink>
        ))}
      </LeftHeader>
      <RightHeader>
        <StyledLogo src="assets/images/logo.png" />
      </RightHeader>
    </HeaderGrid>
  )
}
