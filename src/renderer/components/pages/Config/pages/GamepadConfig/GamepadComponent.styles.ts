import { styled } from '@/renderer/globalStyles/styled'

export const StyledGamepadContainer = styled.div``

export const StyledGamepad = styled.div`
  padding-top: 128px;
  margin: 0 auto;
  position: relative;
  display: table;
  width: 800px;

  img {
    position: relative;
    top: 15%;
    left: 50%;
    transform: translateX(-50%);
    width: 500px;
    height: 400px;
    z-index: 1;
  }
`
