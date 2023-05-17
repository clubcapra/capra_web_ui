const MIN_VOLTAGE = 16.5;
const MAX_VOLTAGE = 20.4;

// Get the battery percentage estimate from the last x voltages
export function getBatteryPercentage(voltages: number[]) {
  // Get the average voltage
  const averageVoltage = voltages.reduce((a, b) => a + b, 0) / voltages.length;

  // Get the percentage
  const percentage = Math.round(
    ((averageVoltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100
  );

  return Math.max(0, percentage);
}
