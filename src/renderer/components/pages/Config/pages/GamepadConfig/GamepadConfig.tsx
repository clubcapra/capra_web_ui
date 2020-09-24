import React, { FC } from 'react'
import { FaSave, FaTimes, FaPlus } from 'react-icons/fa'
import {
  GamepadConfigWrapper,
  FakeProfileList,
  FakeProfileHeader,
  FakeProfile,
  FakeAddProfile,
} from '~components/pages/Config/pages/GamepadConfig/GamepadConfig.styles'
import { GamepadComponent } from '~components/pages/Config/pages/GamepadConfig/GamepadComponent'

const FakeProfileConfig: FC = () => (
  <FakeProfileList>
    <FakeProfileHeader>
      <span>Profiles</span>
    </FakeProfileHeader>
    <FakeProfile>
      <span>Default</span>
      <FaSave />
      <FaTimes />
    </FakeProfile>
    <FakeProfile selected>
      <span>Charles</span>
      <FaSave />
      <FaTimes />
    </FakeProfile>
    <FakeProfile>
      <span>Bob</span>
      <FaSave />
      <FaTimes />
    </FakeProfile>
    <FakeAddProfile>
      <FaPlus />
    </FakeAddProfile>
  </FakeProfileList>
)

export const GamepadConfig = () => {
  return (
    <GamepadConfigWrapper>
      <div style={{ visibility: 'hidden' }}>
        <FakeProfileConfig />
      </div>
      <GamepadComponent />
    </GamepadConfigWrapper>
  )
}
