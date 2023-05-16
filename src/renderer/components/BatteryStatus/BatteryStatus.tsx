import BatteryGauge from 'react-battery-gauge';
import React, { memo } from 'react';
import { styled } from '@/renderer/globalStyles/styled';
import Popup from 'reactjs-popup';
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

const StyledPopup = styled(Popup)`
  @keyframes anvil {
    0% {
      transform: scale(1) translateY(0px);
      opacity: 0;
      box-shadow: 0 0 0 rgba(241, 241, 241, 0);
    }
    1% {
      transform: scale(0.96) translateY(10px);
      opacity: 0;
      box-shadow: 0 0 0 rgba(241, 241, 241, 0);
    }
    100% {
      transform: scale(1) translateY(0px);
      opacity: 1;
      box-shadow: 0 0 500px rgba(241, 241, 241, 0);
    }
  }
  &-content {
    -webkit-animation: anvil 0.2s cubic-bezier(0.38, 0.1, 0.36, 0.9) forwards;
  }
`;

export default memo(BatteryStatus);
