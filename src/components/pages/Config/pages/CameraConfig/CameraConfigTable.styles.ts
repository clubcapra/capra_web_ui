import { styled } from 'globalStyles/styled'

export const StyledCameraConfigTable = styled.table`
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
