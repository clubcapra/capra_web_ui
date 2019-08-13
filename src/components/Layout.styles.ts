import { styled } from 'globalStyles/styled'

export const GridLayout = styled.div`
  height: calc(100% - 1px);
  display: grid;
  grid-template-areas:
    'h e'
    'v e'
    's e';
  grid-template-columns: 16fr 1fr;
  grid-template-rows: 2fr 42fr 1fr;
`

export const StyledView = styled.div`
  grid-area: v;
  height: 100%;
  overflow-y: auto;
`

export const StatusBarArea = styled.div`
  grid-area: s;
`

export const EStopArea = styled.div`
  grid-area: e;
`

export const HeaderArea = styled.div`
  grid-area: h;
`
