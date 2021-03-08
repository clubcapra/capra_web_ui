import { styled } from '@/renderer/globalStyles/styled'

export const ConfigPageGrid = styled.div`
  display: grid;
  grid-template: 'm r';
  grid-template-columns: 1fr 5fr;
  height: 100%;
  max-height: 100%;
`

export const MenuArea = styled.div`
  grid-area: m;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  margin: 0;
  min-height: 100%;
`

export const ConfigRouterArea = styled.div`
  grid-area: r;
  padding: 16px;
  overflow: auto;
`
