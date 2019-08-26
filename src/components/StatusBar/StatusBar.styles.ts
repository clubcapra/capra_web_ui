import { styled } from 'globalStyles/styled'

export const StyledStatusBarWrapper = styled.div`
  display: grid;
  grid-template: 'l r';
  grid-template-columns: auto 1fr;
  height: 100%;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  color: ${({ theme }) => theme.colors.fontLight};
  box-shadow: 0 -2px 2px rgba(0, 0, 0, 0.25);
  font-size: 14px;
`

export const LeftStatusBar = styled.div`
  grid-area: l;
  display: flex;
  align-items: center;
  padding: 0 4px;
`

export const RightStatusBar = styled.div`
  grid-area: r;
  display: flex;
  justify-items: center;
  justify-content: flex-end;

  > * {
    padding: 0 4px;
  }
`
