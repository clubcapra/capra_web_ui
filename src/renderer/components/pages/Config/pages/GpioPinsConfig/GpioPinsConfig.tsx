import React from 'react';
import { SectionTitle } from '../../styles';
import { useDispatch, useSelector } from 'react-redux';
import { selectAllGpioPins } from '@/renderer/store/modules/gpioPins';
import { GpioPin } from './GpioPin';
import { styled } from '@/renderer/globalStyles/styled';

const Container = styled.div`
  display: flex;
  flex-direction: row;
  align-items: flex-start;
`;

export const GpioPinsConfig = () => {
  const dispatch = useDispatch();
  const gpioPins = useSelector(selectAllGpioPins);
  return (
    <div>
      <SectionTitle>GPIO Control</SectionTitle>
      <Container>
        {gpioPins.map((gpioPin) => (
          <GpioPin key={gpioPin.id} gpioPin={gpioPin} />
        ))}
      </Container>
    </div>
  );
};
