import React, { FC, useState } from 'react'
import { styled } from 'globalStyles/styled'
import { darken } from 'polished'
import { Modal } from './common/Modal'
import { Button } from './common/Button'
import { rosClient } from 'utils/ros/rosClient'

export const StyledStopButton = styled.div`
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;
  user-select: none;
  background-color: ${({ theme }) => theme.colors.primary};
  color: ${({ theme }) => theme.colors.fontLight};

  &:hover {
    box-shadow: inset 0 0 2px #000000;
  }

  &:active {
    box-shadow: inset 0 0 6px #000000;
    background-color: ${({ theme }) => darken(0.05, theme.colors.primary)};
  }

  span {
    font-weight: bold;
    font-size: 1.8em;
    writing-mode: vertical-rl;
    text-orientation: upright;
  }
`

export const EStopButton: FC = () => {
  const [isModalOpen, setIsModalOpen] = useState(false)

  const stopRobot = () => {
    rosClient.callService({ name: 'takin_estop_disable', serviceType: '' }, '')
    setIsModalOpen(true)
  }

  const closeModal = () => {
    setIsModalOpen(false)
  }
  const restartRobot = () => {
    rosClient.callService({ name: 'takin_estop_enable', serviceType: '' }, '')
    closeModal()
  }

  return (
    <>
      <StyledStopButton onClick={stopRobot}>
        <span>EMERGENCY STOP</span>
      </StyledStopButton>
      <Modal isOpen={isModalOpen} onClose={closeModal}>
        <h2>Warning!</h2>
        <p>Robot is currently stopped</p>
        <p>Do you want to restart it?</p>
        <div style={{ display: 'flex' }}>
          <Button onClick={restartRobot} btnType="success">
            Yes
          </Button>
          <Button onClick={closeModal} btnType="danger">
            No
          </Button>
        </div>
      </Modal>
    </>
  )
}
