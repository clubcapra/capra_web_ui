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

const Card = styled.div`
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  border-radius: 4px;
  padding: 16px;
  margin: 8px;
  min-width: 150px;
  display: flex;
  flex-direction: column;
  justify-content: flex-start;
  align-items: flex-start;
  box-shadow: 0 0 10px 0 rgba(0, 0, 0, 0.5);
`;

export const GpioBPMPin = ({ gpioPin }: GpioPinProps) => {
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

  const setVal = (event: React.ChangeEvent<HTMLInputElement>) => {
    rosClient.publish(topic, { data: parseInt(event.target.value) });
    dispatch(
      gpioPinsSlice.actions.updateBPM({
        id: gpioPin.id,
        bpm: parseInt(event.target.value, 10),
      })
    );
  };

  return (
    <Card>
      <h3 style={{ paddingBottom: '4px' }}>{gpioPin.name}</h3>
      <input
        type="number"
        min="1"
        max="300"
        value={gpioPin.bpm}
        onInput={setVal}
      />
      <IconButton
        icon={<FaPowerOff />}
        title={gpioPin.isOn ? 'Actuateur désactivé' : 'Actuateur activé'}
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
