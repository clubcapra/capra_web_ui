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

export const CameraConfigTable = styled.table`
  width: 100%;
  border-spacing: 0;

  th,
  td {
    text-align: left;
    padding: 8px 8px;

    &:last-child {
      width: 32px;
    }
  }

  td:last-child {
    cursor: pointer;
  }

  thead th {
    border-bottom: 1px solid ${({ theme }) => theme.colors.border};
  }
`

export const StyledTableInput = styled.input`
  height: 32px;
  padding: 4px 8px;
  background-color: ${({ theme }) => theme.colors.foreground};
  border: 1px solid ${({ theme }) => theme.colors.border};
  color: ${({ theme }) => theme.colors.fontLight};
  border-radius: 2px;

  &:active,
  &:focus {
    outline-color: ${({ theme }) => theme.colors.primary};
  }
`
