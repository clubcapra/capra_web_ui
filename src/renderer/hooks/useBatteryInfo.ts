import { useCallback, useMemo, useState } from 'react';
import { useRosSubscribe } from './useRosSubscribe';
import { getBatteryPercentage } from '../utils/math/battery';

interface BatteryInfoTopic {
  name: string;
  messageType: string;
}

interface BatteryInfo {
  percentage: number;
  voltage: number;
}

const useBatteryInfo = (topicName: string) => {
  const [batteryInfo, setBatteryInfo] = useState<BatteryInfo>({
    percentage: 0,
    voltage: 0,
  });

  // Store the last 100 voltages to smooth out the battery percentage
  const [lastVoltages] = useState<number[]>([]);

  const topic: BatteryInfoTopic = useMemo(
    () => ({
      name: topicName,
      messageType: 'std_msgs/Float64',
    }),
    [topicName]
  );

  useRosSubscribe<BatteryInfoTopic>(
    topic,
    useCallback(
      (message) => {
        const voltage = Number(message.data);

        lastVoltages.push(voltage);
        // Don't update the battery percentage if the difference between the last two voltages is less than 0.1
        if (Math.abs(voltage - lastVoltages[lastVoltages.length - 2]) > 0.1) {
          const percentage = getBatteryPercentage(lastVoltages);

          setBatteryInfo({
            percentage,
            voltage,
          });
        }
        // Remove the first voltage if we have more than 100
        if (lastVoltages.length > 100) {
          lastVoltages.shift();
        }
      },
      [lastVoltages]
    )
  );

  return batteryInfo;
};

export default useBatteryInfo;
