import { styled } from '@/renderer/globalStyles/styled'
import { NavLink } from 'react-router-dom'
import { darken } from 'polished'

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
    background-color: ${({ theme }) => darken(0.05, theme.colors.background)};
  }

  &.is-active {
    background-color: ${({ theme }) => theme.colors.background};
  }
`
