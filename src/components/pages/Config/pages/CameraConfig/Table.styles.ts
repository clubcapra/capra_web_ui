import { styled } from 'globalStyles/styled'
import { darken } from 'polished'

export const StyledTable = styled.table`
  width: 100%;
  border-spacing: 0;
  border-collapse: collapse;

  th,
  td {
    text-align: left;

    &:last-child {
      width: 32px;
    }
  }

  td {
    padding: 8px 8px;

    &:last-child {
      cursor: pointer;
    }
  }

  thead th {
    padding: 4px 8px;
    font-size: inherit;
    border-bottom: 1px solid ${({ theme }) => theme.colors.border};
  }

  tbody {
    border: 1px solid ${({ theme }) => theme.colors.border};
  }
`

export const StyledTableInput = styled.input`
  height: 32px;
  width: 100%;
  padding: 4px 8px;
  background-color: ${({ theme }) => theme.colors.foreground};
  border: 1px solid ${({ theme }) => theme.colors.border};
  color: ${({ theme }) => theme.colors.fontLight};

  &:active,
  &:focus {
    outline-color: ${({ theme }) => theme.colors.primary};
  }

  &:disabled {
    background-color: ${({ theme }) => darken(0.1, theme.colors.foreground)};
    color: ${({ theme }) => darken(0.2, theme.colors.fontLight)};
    cursor: not-allowed;
  }
`
