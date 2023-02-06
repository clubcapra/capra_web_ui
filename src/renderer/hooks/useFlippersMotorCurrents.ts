import { useCallback, useState } from 'react';
import { selectRobotNameState } from '../store/modules/input';

import { store } from '../store/store';
import { useRosSubscribe } from './useRosSubscribe';

interface FlipperTopic {
  name: string;
  messageType: string;
}

const flipperFL = {
  name: `markhor/flippers/flipper_fl_motor_current`,
  messageType: 'std_msgs/Float64',
};
const flipperFR = {
  name: `markhor/flippers/flipper_fr_motor_current`,
  messageType: 'std_msgs/Float64',
};
const flipperRL = {
  name: `markhor/flippers/flipper_rl_motor_current`,
  messageType: 'std_msgs/Float64',
};
const flipperRR = {
  name: `markhor/flippers/flipper_rr_motor_current`,
  messageType: 'std_msgs/Float64',
};

export const useFlippersMotorCurrents = () => {
  const robotName = selectRobotNameState(store.getState());

  const [flMotorCurrentState, setFlMotorCurrentState] = useState(0);
  const [frMotorCurrentState, setFrMotorCurrentState] = useState(0);
  const [rlMotorCurrentState, setRlMotorCurrentState] = useState(0);
  const [rrMotorCurrentState, setRrMotorCurrentState] = useState(0);

  // useRosSubscribe for each flipper
  useRosSubscribe<FlipperTopic>(
    flipperFL,
    useCallback((message) => {
      const motorCurrent = Number(message.data);
      setFlMotorCurrentState(motorCurrent);
    }, [])
  );

  useRosSubscribe<FlipperTopic>(
    flipperFR,
    useCallback((message) => {
      const motorCurrent = Number(message.data);
      setFrMotorCurrentState(motorCurrent);
    }, [])
  );

  useRosSubscribe<FlipperTopic>(
    flipperRL,
    useCallback((message) => {
      const motorCurrent = Number(message.data);
      setRlMotorCurrentState(motorCurrent);
    }, [])
  );

  useRosSubscribe<FlipperTopic>(
    flipperRR,
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
