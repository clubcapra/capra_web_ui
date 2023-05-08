import IconButton from '@/renderer/components/common/IconButton';
import { styled } from '@/renderer/globalStyles/styled';
import { GpioPinsState } from '@/renderer/store/modules/gpioPins';
import React from 'react';
import { FaPowerOff } from 'react-icons/fa';

interface GpioPinProps {
  gpioPin: GpioPinsState;
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
  transition: box-shadow 0.2s ease-in-out;
`;

export const GpioPin = ({ gpioPin }: GpioPinProps) => {
  return (
    <Card>
      <h3>{gpioPin.name}</h3>
      <IconButton
        icon={<FaPowerOff />}
        title={gpioPin.isOn ? 'Power Off' : 'Power On'}
        onClick={() => {}}
        style={
          gpioPin.isOn
            ? { backgroundColor: 'red' }
            : { backgroundColor: 'green' }
        }
      />
    </Card>
  );
};
