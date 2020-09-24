import { styled } from '~globalStyles/styled'

interface StyledFeedProps {
  width?: string
  height?: string
}

export const StyledFeedComponent = styled.div<StyledFeedProps>`
  position: relative;
  display: flex;
  justify-content: center;
  align-items: center;
  height: 100%;
  width: 100%;
  padding: 1px;
`
