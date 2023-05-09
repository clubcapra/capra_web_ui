import BatteryGauge from 'react-battery-gauge';
import React from 'react';
import { styled } from '@/renderer/globalStyles/styled';

interface BatteryStatusProps {
  name?: string;
}

const BatteryStatus = ({ name }: BatteryStatusProps) => {
  const customization = {
    batteryMeter: {
      lowBatteryValue: 40,
    },
    readingText: {
      opacity: 0,
    },
    batteryBody: {
      strokeColor: '#fff',
      strokeWidth: 2,
    },
    batteryCap: {
      strokeColor: '#fff',
      strokeWidth: 2,
    },
  };
  const batteryLevel = 70;
  return (
    <Container>
      <PercentageText>{name}</PercentageText>
      <PercentageText>{batteryLevel}%</PercentageText>
      <BatteryGauge
        value={batteryLevel}
        size={40}
        aspectRatio={0.42}
        customization={customization}
      />
    </Container>
  );
};

const Container = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: center;
  align-items: center;
`;

const PercentageText = styled.div`
  font-size: 12px;
  margin-right: 5px;
  font-weight: bold;
`;

export default BatteryStatus;
