import React, { FC } from 'react'
import { styled } from '@/renderer/globalStyles/styled'
import { NavLink } from 'react-router-dom'

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
  {
    to: '/debug',
    label: 'Debug',
  },
]

export const Header: FC = () => {
  return (
    <HeaderGrid>
      <LeftHeader navlinks={navLinks}>
        {navLinks.map(({ to, label }) => (
          <StyledNavLink
            key={to}
            to={to}
            activeClassName="is-active"
            id={label}
            style={
              label.includes('Debug')
                ? { display: 'none' }
                : { display: 'grid' }
            }
          >
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

const HeaderGrid = styled.div`
  display: grid;
  grid-template-columns: 1fr 100px;
  box-shadow: 0 3px 2px rgba(0, 0, 0, 0.25);
`

const LeftHeader = styled.div<{
  navlinks: NavLinkDefinition[]
}>`
  display: grid;
  grid-template-columns: ${({ navlinks }) => navlinks.map(() => '100px ')};
  justify-items: center;
`

const RightHeader = styled.div`
  margin: 2px;
`

const StyledNavLink = styled(NavLink)`
  display: grid;
  width: 100%;
  align-items: center;
  text-align: center;
  text-decoration: none;
  color: ${({ theme }) => theme.colors.fontLight};
  border-bottom-color: ${({ theme }) => theme.colors.primary};

  &:hover {
    font-weight: bold;
  }

  &.is-active {
    background-color: ${({ theme }) => theme.colors.darkerBackground};
    color: ${({ theme }) => theme.colors.fontLight};
    border-bottom: 2px solid ${({ theme }) => theme.colors.primary};
  }
`

const StyledLogo = styled.img`
  height: auto;
  max-width: 100%;
`
