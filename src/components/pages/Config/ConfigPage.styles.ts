import { styled } from 'globalStyles/styled'

export const ConfigPageGrid = styled.div`
  display: grid;
  grid-template: 'm r';
  grid-template-columns: 1fr 5fr;
  position: relative;
  height: 99%;
`

export const MenuArea = styled.div`
  grid-area: m;
  border-right: 1px solid ${({ theme }) => theme.colors.border};
  margin: 0;
  min-height: 100%;
`

export const ConfigRouterArea = styled.div`
  grid-area: r;
  padding: 16px;
  overflow: auto;
`
