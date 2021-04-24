import * as React from 'react'
import { ShortcutBox } from '@/renderer/components/pages/Config/pages/GamepadConfig/ShortcutBox'
import { styled } from '@/renderer/globalStyles/styled'

interface ShortcutData {
  text: string
  from: [number, number]
  to: [number, number]
}

const data: ShortcutData[] = [
  { text: 'Move Robot', from: [0, 250], to: [268, 298] }, // LS
  // { text: 'Move Arm', from: [700, 400], to: [455, 370] }, // RS
  { text: 'Brake', from: [0, 100], to: [275, 160] }, // LT
  { text: 'Throttle', from: [680, 100], to: [505, 160] }, // RT
  { text: 'Headlights', from: [0, 350], to: [300, 375] }, // DPAD
  { text: 'Toggle control mode', from: [0, 300], to: [345, 375] }, // DPAD
  // { text: 'Switch Camera', from: [700, 230], to: [515, 265] }, // Y
  // { text: 'Stop', from: [700, 280], to: [550, 295] }, // B
  { text: 'Movement enable', from: [680, 330], to: [515, 330] }, // A
]

export const GamepadComponent: React.FC = () => {
  return (
    <StyledGamepad>
      {data.map((x) => (
        <ShortcutBox key={x.text} from={x.from} to={x.to}>
          {x.text}
        </ShortcutBox>
      ))}
      <img src="assets/images/xbox-one-controller.png" alt="xbox one gamepad" />
    </StyledGamepad>
  )
}

const StyledGamepadContainer = styled.div``

const StyledGamepad = styled.div`
  padding-top: 128px;
  margin: 0 auto;
  position: relative;
  display: table;
  width: 800px;

  img {
    position: relative;
    top: 15%;
    left: 50%;
    transform: translateX(-50%);
    width: 500px;
    height: 400px;
    z-index: 1;
  }
`
