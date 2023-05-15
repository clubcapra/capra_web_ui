import React from 'react';
import { styled } from '@/renderer/globalStyles/styled';
import BatteryGauge from 'react-battery-gauge';
import { defaultTheme } from '@/renderer/globalStyles/themes/defaultTheme';

interface BatteryDetailsPopupProps {
  batteryValue: number;
  voltage: number;
  lowBatteryValue: number;
}

const Card = styled.div`
  width: 200px;
  height: 200px;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  border-radius: 10px;
`;

const BatteryContainer = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: center;
  align-items: center;
  height: 60%;
  border-radius: 10px 10px 0 0;
`;

const BatteryInfo = styled.div`
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  height: 40%;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  border-radius: 0 0 10px 10px;
`;

const PercentageText = styled.p`
  margin: 0;
  padding: 0;
  font-size: 28px;
  color: ${({ theme }) => theme.colors.fontLight};
  font-weight: 700;
`;

const customization = {
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
  batteryMeter: {
    noOfCells: 10,
    fill: '#fff',
  },
};

const BatteryDetailsPopup = ({
  batteryValue,
  voltage,
  lowBatteryValue,
}: BatteryDetailsPopupProps) => {
  return (
    <Card>
      <BatteryContainer
        style={{
          backgroundColor:
            lowBatteryValue < batteryValue
              ? defaultTheme.colors.success
              : defaultTheme.colors.danger,
        }}
      >
        <BatteryGauge
          value={batteryValue}
          size={100}
          orientation="vertical"
          customization={customization}
        />
      </BatteryContainer>
      <BatteryInfo>
        <PercentageText>{batteryValue}%</PercentageText>
        <p>Voltage: {voltage}V</p>
      </BatteryInfo>
    </Card>
  );
};

export default BatteryDetailsPopup;
