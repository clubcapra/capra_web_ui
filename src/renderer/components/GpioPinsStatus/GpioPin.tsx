import IconButton from '@/renderer/components/common/IconButton';
import { styled } from '@/renderer/globalStyles/styled';
import { GpioPinState, gpioPinsSlice } from '@/renderer/store/modules/gpioPins';
import { rosClient } from '@/renderer/utils/ros/rosClient';
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types';
import React, { useMemo } from 'react';
import { FaPowerOff } from 'react-icons/fa';
import { useDispatch } from 'react-redux';

interface GpioPinProps {
  gpioPin: GpioPinState;
}

export const GpioPin = ({ gpioPin }: GpioPinProps) => {
  const dispatch = useDispatch();

  const topic: TopicOptions = useMemo(
    () => ({
      name: gpioPin.topicName,
      messageType: 'std_msgs/UInt16',
    }),
    [gpioPin.topicName]
  );

  const togglePin = () => {
    rosClient.publish(topic, { data: gpioPin.isOn ? 0 : 1 });

    dispatch(gpioPinsSlice.actions.togglePin(gpioPin.id));
  };

  return (
    <Card>
      <SytledP>{gpioPin.name}</SytledP>
      <IconButton
        icon={<FaPowerOff />}
        title={gpioPin.isOn ? 'Power Off' : 'Power On'}
        onClick={togglePin}
        style={
          gpioPin.isOn
            ? { backgroundColor: 'red' }
            : { backgroundColor: 'green' }
        }
      />
    </Card>
  );
};

const Card = styled.div`
  padding: 8px;
  margin: 4px;
  min-width: 150px;
  display: flex;
  flex-direction: column;
  justify-content: flex-start;
`;

const SytledP = styled.p`
  margin-bottom: 0.25em;
`;
