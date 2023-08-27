import React from 'react';
import { SectionTitle } from '../../styles';
import { useSelector } from 'react-redux';
import { selectAllGpioPins } from '@/renderer/store/modules/gpioPins';
import { GpioPin } from './GpioPin';
import { GpioBPMPin } from './GpioBPMPin';
import { styled } from '@/renderer/globalStyles/styled';

const Container = styled.div`
  display: flex;
  flex-direction: row;
  align-items: flex-start;
`;

export const GpioPinsConfig = () => {
  const gpioPins = useSelector(selectAllGpioPins);
  return (
    <>
      <SectionTitle>GPIO Control</SectionTitle>
      <Container>
        {gpioPins.map((gpioPin) =>
          gpioPin.bpm ? (
            <GpioBPMPin key={gpioPin.id} gpioPin={gpioPin} />
          ) : (
            <GpioPin key={gpioPin.id} gpioPin={gpioPin} />
          )
        )}
      </Container>
    </>
  );
};
