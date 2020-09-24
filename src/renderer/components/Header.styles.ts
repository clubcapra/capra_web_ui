import { styled } from '~globalStyles/styled'
import { NavLink } from 'react-router-dom'

export const HeaderGrid = styled.div`
  display: grid;
  grid-template-columns: 1fr 100px;
  box-shadow: 0 3px 2px rgba(0, 0, 0, 0.25);
`

interface LeftHeaderProps {
  navlinks: unknown[]
}

export const LeftHeader = styled.div<LeftHeaderProps>`
  display: grid;
  grid-template-columns: ${({ navlinks }) => navlinks.map(() => '100px ')};
  justify-items: center;
`

export const RightHeader = styled.div`
  margin: 2px;
`

export const StyledNavLink = styled(NavLink)`
  display: grid;
  width: 100%;
  align-items: center;
  text-align: center;
  text-decoration: none;
  color: ${({ theme }) => theme.colors.fontLight};
  transition: background-color 0.2s ease, border-bottom 0.2s ease-in-out;
  transition-delay: 0.2s;
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

export const StyledLogo = styled.img`
  height: auto;
  max-width: 100%;
`
