import { styled } from '@/renderer/globalStyles/styled'

//TODO 70 px is the width of the estop
export const StyledOverlay = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  width: calc(100% - 70px);
  height: 55%;
  z-index: 997;
`

export const StyledTerm = styled.div`
  border: 1px solid rgba(0, 255, 0, 0.3);
  height: 100%;
  width: 100&;
`
