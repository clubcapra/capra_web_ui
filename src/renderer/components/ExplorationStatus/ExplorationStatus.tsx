import { styled } from '@/renderer/globalStyles/styled';
import { ExplorationCancelNavigation } from './ExplorationCancelNavigation';
import { GoTelescope } from 'react-icons/go';
import { Countdown } from '@/renderer/components/common/Countdown';
import { rosClient } from '@/renderer/utils/ros/rosClient';
import React, { FC, useState } from 'react';
import { log } from '@/renderer/logger';

export const ExplorationStatus: FC = () => {
  const [isNowStopCountdownTimer, setIsNowStopCountdownTimer] = useState(false);

  const startTimer = (duration: number) => {
    setIsNowStopCountdownTimer(false);
    startRosExplorationTimer(duration);
  };

  const stopTimer = () => {
    setIsNowStopCountdownTimer(true);
    stopRosExplorationTimer();
  };

  const startRosExplorationTimer = (duration: number) => {
    rosClient
      .callService(
        {
          name: `/start_exploration`,
        },
        { timeout: duration * 60 }
      )
      .catch(log.error);
  };

  const stopRosExplorationTimer = () => {
    rosClient
      .callService(
        {
          name: `/stop_exploration`,
        },
        {}
      )
      .catch(log.error);
  };

  return (
    <Countdown
      icon={<StyledGoTelescope />}
      labelPopup={'exploration'}
      durationDefault={2}
      onStartClick={startTimer}
      onStopClick={stopTimer}
      sideElement={
        <ExplorationCancelNavigation isCancelNavigationProps={stopTimer} />
      }
      isNowStopCountdownTimer={isNowStopCountdownTimer}
    />
  );
};

const StyledGoTelescope = styled(GoTelescope)`
  height: 1.25em;
  width: 1.25em;
`;
