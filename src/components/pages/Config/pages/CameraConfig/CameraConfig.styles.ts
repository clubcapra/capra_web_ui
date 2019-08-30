import { styled } from 'globalStyles/styled'

export const CameraConfigWrapper = styled.div`
  width: 100%;
  height: 100%;
  max-width: 800px;
`

export const CameraConfigHeaderWrapper = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;

  h1 {
    margin: 0;
  }
`

export const CameraAddButton = styled.div`
  padding: 4px 8px;
  background-color: ${({ theme }) => theme.colors.primary};
  border-radius: 2px;
  cursor: pointer;
  transition: all 0.1s ease;
  user-select: none;

  &:hover {
    background-color: ${({ theme }) => theme.colors.accent};
    color: ${({ theme }) => theme.colors.fontDark};
  }

  &:active {
    transition: none;
    background-color: ${({ theme }) => theme.colors.primary};
    color: ${({ theme }) => theme.colors.fontLight};
  }
`
