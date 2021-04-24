import { styled } from '@/renderer/globalStyles/styled'

export const StyledModal = styled.div`
  height: auto;
  width: auto;
  min-width: 500px;
  position: fixed;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  z-index: 999;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.32), 0 2px 6px rgba(0, 0, 0, 0.21);
  background-color: ${({ theme }) => theme.colors.background};
`

export const StyledModalHeader = styled.div`
  display: flex;
  border-bottom: 1px solid ${({ theme }) => theme.colors.darkerBackground};
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.32), 0 2px 6px rgba(0, 0, 0, 0.21);
`

export const StyledModalTitle = styled.h3`
  padding: 5px;
  flex-grow: 9;
`

export const StyledModalClose = styled.div`
  flex-grow: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;

  &:hover {
    background-color: red;
  }

  svg {
    height: 24px;
    width: 24px;
  }
`

export const StyledModalContent = styled.div`
  padding: 5px;
`

export const StyledOverlay = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  width: 100vw;
  height: 100vh;
  background-color: rgba(0, 0, 0, 0.4);
  z-index: 998;
`
