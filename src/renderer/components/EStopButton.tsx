import React, { FC, useState } from 'react'
import { Modal } from './common/Modal'
import { Button } from './common/Button'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import { useOpenClose } from '@/renderer/hooks/useOpenClose'
import { styled } from '@/renderer/globalStyles/styled'
import { darken } from 'polished'
import { log } from '@/renderer/logger'
import { selectRobotName } from '../store/modules/ros'
import { useSelector } from '@/renderer/hooks/typedUseSelector'

interface StopButtonProps {
  onClick: () => void
}

const StopButton: FC<StopButtonProps> = ({ onClick }) => {
  const [text] = useState('EMERGENCY STOP')

  return (
    <StyledStopButton onClick={onClick}>
      <span>{text}</span>
    </StyledStopButton>
  )
}

//TODO change robotName for 'capra' or an organisation field equivalent. Both UI side and in the estop code.
export const EStopButton: FC = () => {
  const [isModalOpen, openModal, closeModal] = useOpenClose()
  const robotName = useSelector(selectRobotName)

  const stopRobot = () => {
    log.info('ESTOP: stopping robot')
    rosClient
      .callService({ name: { robotName } + '/estop_disable', serviceType: '' }, '')
      .catch(log.error)
    openModal()
  }

  const restartRobot = () => {
    log.info('ESTOP: restarting robot')
    rosClient
      .callService({ name: { robotName } + '/estop_enable', serviceType: '' }, '')
      .catch(log.error)
    closeModal()
  }

  return (
    <>
      <StopButton onClick={stopRobot} />
      <Modal
        isOpen={isModalOpen}
        onClose={closeModal}
        title={'Warning!'}
        footer={
          <div style={{ display: 'flex' }}>
            <Button onClick={restartRobot} btnType="success">
              Yes
            </Button>
            <Button onClick={closeModal} btnType="danger">
              No
            </Button>
          </div>
        }
      >
        <p>Robot is currently stopped.</p>
        <p>Do you want to restart it?</p>
      </Modal>
    </>
  )
}

const StyledStopButton = styled.div`
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
