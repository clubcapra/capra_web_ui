import React, { FC, useState } from 'react'
import { Modal } from './common/Modal'
import { Button } from './common/Button'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import { StyledStopButton } from './EStopButton.styles'
import { useRosSubscribe } from '@/renderer/utils/hooks/useRosSubscribe'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import { useOpenClose } from '@/renderer/utils/hooks/useOpenClose'

const topic: TopicOptions<boolean> = {
  name: 'takin_estop_status',
  messageType: 'std_msgs/Bool',
}

interface StopButtonProps {
  onClick: () => void
}

const StopButton: FC<StopButtonProps> = ({ onClick }) => {
  const [text, setText] = useState('EMERGENCY STOP')

  useRosSubscribe(topic, (message) => {
    if (message.data) {
      setText('EMERGENCY STOP')
    } else {
      setText('REARM')
    }
  })

  return (
    <StyledStopButton onClick={onClick}>
      <span>{text}</span>
    </StyledStopButton>
  )
}

export const EStopButton: FC = () => {
  const [isModalOpen, openModal, closeModal] = useOpenClose()

  const stopRobot = () => {
    rosClient.callService({ name: 'takin_estop_disable', serviceType: '' }, '')
    openModal()
  }

  const restartRobot = () => {
    rosClient.callService({ name: 'takin_estop_enable', serviceType: '' }, '')
    closeModal()
  }

  return (
    <>
      <StopButton onClick={stopRobot} />
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
