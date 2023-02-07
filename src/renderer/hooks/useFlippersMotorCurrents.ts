import { useCallback, useMemo, useState } from 'react';
import { selectRobotNameState } from '../store/modules/input';

import { store } from '../store/store';
import { useRosSubscribe } from './useRosSubscribe';

interface FlipperTopic {
  name: string;
  messageType: string;
}

interface FlipperTopics {
  flipperFLTopic: FlipperTopic;
  flipperFRTopic: FlipperTopic;
  flipperRLTopic: FlipperTopic;
  flipperRRTopic: FlipperTopic;
}

export const useFlippersMotorCurrents = () => {
  const robotName = selectRobotNameState(store.getState());

  const [flMotorCurrentState, setFlMotorCurrentState] = useState(0);
  const [frMotorCurrentState, setFrMotorCurrentState] = useState(0);
  const [rlMotorCurrentState, setRlMotorCurrentState] = useState(0);
  const [rrMotorCurrentState, setRrMotorCurrentState] = useState(0);
  const flipperTopics = useMemo<FlipperTopics>(
    () => ({
      flipperFLTopic: {
        name: `${robotName}/flippers/flipper_fl_motor_current`,
        messageType: 'std_msgs/Float64',
      },
      flipperFRTopic: {
        name: `${robotName}/flippers/flipper_fr_motor_current`,
        messageType: 'std_msgs/Float64',
      },
      flipperRLTopic: {
        name: `${robotName}/flippers/flipper_rl_motor_current`,
        messageType: 'std_msgs/Float64',
      },
      flipperRRTopic: {
        name: `${robotName}/flippers/flipper_rr_motor_current`,
        messageType: 'std_msgs/Float64',
      },
    }),
    [robotName]
  );

  // useRosSubscribe for each flipper
  useRosSubscribe<FlipperTopic>(
    flipperTopics.flipperFLTopic,
    useCallback((message) => {
      const motorCurrent = Number(message.data);
      setFlMotorCurrentState(motorCurrent);
    }, [])
  );

  useRosSubscribe<FlipperTopic>(
    flipperTopics.flipperFRTopic,
    useCallback((message) => {
      const motorCurrent = Number(message.data);
      setFrMotorCurrentState(motorCurrent);
    }, [])
  );

  useRosSubscribe<FlipperTopic>(
    flipperTopics.flipperRLTopic,
    useCallback((message) => {
      const motorCurrent = Number(message.data);
      setRlMotorCurrentState(motorCurrent);
    }, [])
  );

  useRosSubscribe<FlipperTopic>(
    flipperTopics.flipperRRTopic,
    useCallback((message) => {
      const motorCurrent = Number(message.data);
      setRrMotorCurrentState(motorCurrent);
    }, [])
  );

  return {
    flMotorCurrentState,
    frMotorCurrentState,
    rlMotorCurrentState,
    rrMotorCurrentState,
  };
};
