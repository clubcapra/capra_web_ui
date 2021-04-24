import React from 'react'
import { GamepadComponent } from '@/renderer/components/pages/Config/pages/GamepadConfig/GamepadComponent'
import { styled } from '@/renderer/globalStyles/styled'

export const GamepadConfig = () => {
  return (
    <GamepadConfigWrapper>
      <GamepadComponent />
    </GamepadConfigWrapper>
  )
}

const GamepadConfigWrapper = styled.div`
  width: 100%;
  display: flex;
  justify-content: center;
  align-items: center;
  position: relative;
`
