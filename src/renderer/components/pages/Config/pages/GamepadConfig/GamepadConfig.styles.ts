import { styled } from '@/renderer/globalStyles/styled'

export const GamepadConfigWrapper = styled.div`
  width: 100%;
  display: flex;
  justify-content: center;
  align-items: center;
  position: relative;
`

export const FakeProfileList = styled.div`
  position: absolute;
  top: 24px;
  left: 24px;
  width: 200px;
  height: auto;
  min-height: 400px;
  border: 1px solid ${({ theme }) => theme.colors.border};
`

interface FakeProfileProps {
  selected?: boolean
}

export const FakeProfileHeader = styled.div<FakeProfileProps>`
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 8px 0;
  border-bottom: 1px solid ${({ theme }) => theme.colors.border};
  background-color: ${({ theme }) => theme.colors.darkerBackground};
`

export const FakeAddProfile = styled.div<FakeProfileProps>`
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 8px 0;
`

export const FakeProfile = styled.div<FakeProfileProps>`
  display: grid;
  grid-template-columns: 120px 1fr 1fr;
  align-items: center;
  background-color: ${({ selected }) =>
    selected ? 'rgba(0,0,0,0.2)' : 'transparent'};
  padding: 8px 0 8px 8px;
  justify-items: center;

  & > span:first-child {
    justify-self: flex-start;
  }
`
