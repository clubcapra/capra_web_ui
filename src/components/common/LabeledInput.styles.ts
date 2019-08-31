import { styled } from 'globalStyles/styled'

export const StyledLabel = styled.label`
  display: block;
  padding-bottom: 2px;
  /* font-weight: bold; */
  margin-top: 4px;
`

export const StyledInput = styled.input`
  display: block;
  margin-bottom: 8px;

  height: 28px;
  padding: 6px;
  box-shadow: 0 1px 2px rgba(0, 0, 0, 0.1) inset, 0 0 0 3px transparent;

  border-width: 1px;
  border-style: solid;
  border-color: black;

  background-color: ${({ theme }) => theme.colors.darkerBackground};
  color: ${({ theme }) => theme.colors.fontLight};
`
