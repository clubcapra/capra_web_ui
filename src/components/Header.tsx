import React, { FC } from 'react'
import logo from 'assets/images/logo.png'
import {
  HeaderGrid,
  LeftHeader,
  StyledNavLink,
  RightHeader,
  StyledLogo,
} from 'components/Header.styles'

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
  // {
  //   to: '/map',
  //   label: 'Map',
  // },
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
        <StyledLogo src={logo} />
      </RightHeader>
    </HeaderGrid>
  )
}
