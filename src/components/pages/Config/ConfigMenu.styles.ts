import { styled } from 'globalStyles/styled'
import { NavLink } from 'react-router-dom'

export const MenuLabel = styled.p`
  color: ${props => props.theme.colors.fontLight};
  font-size: 0.75em;
  letter-spacing: 0.1em;
  text-transform: uppercase;
  margin: 0;
  border-bottom: 1px solid ${({ theme }) => theme.colors.border};
  padding: 8px 16px 8px;
  background-color: ${({ theme }) => theme.colors.foreground};
`

export const MenuList = styled.ul`
  margin: 0;
  padding: 8px 0 32px;
  line-height: 1.25;
  list-style: none;
  & > li {
    margin: 0;
    padding: 0;
  }
`

export const StyledNavLink = styled(NavLink)`
  display: block;
  padding: 8px 16px;
  color: ${({ theme }) => theme.colors.fontLight};
  text-decoration: none;

  &:hover {
    background-color: ${({ theme }) => theme.colors.darkerBackground};
    color: ${({ theme }) => theme.colors.fontLight};
  }

  &.is-active {
    background-color: ${({ theme }) => theme.colors.primary};
    color: ${({ theme }) => theme.colors.fontLight};
  }
`
