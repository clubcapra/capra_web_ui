import { styled } from '@/renderer/globalStyles/styled'
import React from 'react'
import { FC } from 'react'
import ClipLoader from 'react-spinners/ClipLoader'

export const LoadingOverlay: FC = (props) => {
  const StyledOverlay = styled.div`
    display: flex;
    justify-content: center;
    align-items: center;
    flex-direction: column;
    height: 100%;
    width: 100%;
    background-color: rgba(0, 0, 0, 0.5);
    top: 0;
    position: absolute;
    left: 0;
    z-index: 1000;
  `

  const StyledSpan = styled.span`
    color: white;
    font-size: 1.5rem;
    font-weight: bold;
  `

  return (
    <StyledOverlay>
      <ClipLoader size={100} speedMultiplier={0.5} color={'green'} />
      <StyledSpan>{props.children}</StyledSpan>
    </StyledOverlay>
  )
}
