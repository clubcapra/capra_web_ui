import { useCallback, useState } from 'react';
import { TopicOptions } from '../utils/ros/roslib-ts-client/@types';
import { useRosSubscribeNoData } from './useRosSubscribe';

const jointPositionTopic: TopicOptions = {
  name: 'ovis/arm/out/joint_position',
  messageType: 'ovis_msgs/OvisJointPosition',
};

interface JointPositionMsg {
  joint_positions: number[];
}

const useArmJointPositions = () => {
  const [jointPositions, setJointPositions] = useState<number[]>();

  useRosSubscribeNoData<JointPositionMsg>(
    jointPositionTopic,
    useCallback((message) => {
      setJointPositions(message.joint_positions);
    }, [])
  );

  return jointPositions;
};

export default useArmJointPositions;
