import BatteryGauge from 'react-battery-gauge';
import React, { memo } from 'react';
import { styled } from '@/renderer/globalStyles/styled';
import { StyledPopup } from '@/renderer/components/styles';
import BatteryDetailsPopup from './BatteryDetailsPopup';
import { defaultTheme } from '@/renderer/globalStyles/themes/defaultTheme';
import useBatteryInfo from '@/renderer/hooks/useBatteryInfo';

const LOW_BATTERY_THRESHOLD = 30;

interface BatteryStatusProps {
  name?: string;
  topicName: string;
}

const customization = {
  batteryMeter: {
    lowBatteryValue: LOW_BATTERY_THRESHOLD,
    fill: defaultTheme.colors.success,
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

const BatteryStatus = ({ name, topicName }: BatteryStatusProps) => {
  const batteryInfo = useBatteryInfo(topicName);
  return (
    <StyledPopup
      trigger={
        <Container>
          <PercentageText>{name}</PercentageText>
          <PercentageText>{batteryInfo.percentage.toFixed(0)}%</PercentageText>
          <BatteryGauge
            value={batteryInfo.percentage ?? 0}
            size={40}
            aspectRatio={0.42}
            customization={customization}
          />
        </Container>
      }
      on="click"
      position="bottom center"
      arrow={false}
      repositionOnResize={true}
    >
      <BatteryDetailsPopup
        batteryValue={batteryInfo.percentage ?? 0}
        lowBatteryValue={LOW_BATTERY_THRESHOLD}
        voltage={batteryInfo.voltage ?? 0}
      />
    </StyledPopup>
  );
};

const Container = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: center;
  align-items: center;
  &:hover {
    cursor: pointer;
  }
`;

const PercentageText = styled.div`
  font-size: 12px;
  margin-right: 5px;
  font-weight: bold;
`;

export default memo(BatteryStatus);
