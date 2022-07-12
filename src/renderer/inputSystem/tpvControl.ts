import {
  buttons as buttonMappings,
  sticks,
} from '@/renderer/inputSystem/mappings'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'
import { deadzone } from '../utils/gamepad'

const tpvXTopic: TopicOptions = {
  name: '/tpv_x',
  messageType: 'std_msgs/Float64',
}

const tpvYTopic: TopicOptions = {
  name: '/tpv_y',
  messageType: 'std_msgs/Float64',
}

/**
 * Function that publishes the current TPV values to the ROS topic
 */
export const handleTpvControl = (gamepad: Gamepad) => {
  const tpvEnabled = gamepad.buttons[buttonMappings.LB].pressed
  rosClient.publish(tpvXTopic, {
    data: tpvEnabled
      ? deadzone(-gamepad.axes[sticks.right.horizontal] * 0.5)
      : 0,
  })
  rosClient.publish(tpvYTopic, {
    data: tpvEnabled ? deadzone(gamepad.axes[sticks.right.vertical] * 0.5) : 0,
  })
}
