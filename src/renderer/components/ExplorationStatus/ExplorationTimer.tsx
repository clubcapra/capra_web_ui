import { Button } from '@/renderer/components/common/Button';
import { rosClient } from '@/renderer/utils/ros/rosClient';
import React, { useEffect, FC, useState, ChangeEvent } from 'react';
import { LabeledInput } from '@/renderer/components/common/LabeledInput';
import { log } from '@/renderer/logger';
import { styled } from '@/renderer/globalStyles/styled';

interface TimerDisplayProps {
  setTimerDisplayProps: (timerDisplay: string) => void;
}

export const ExplorationTimer: FC<TimerDisplayProps> = ({
  setTimerDisplayProps,
}) => {
  const timeDisplayDefault = '00:00';

  const [duration, setDuration] = useState(2);
  const [timeRemaining, setTimeRemaining] = useState(0);
  const [isTimerActive, setIsTimerActive] = useState(false);
  const [timerDisplay, setTimerDisplay] = useState(timeDisplayDefault);
  const [countDownDate, setCountDownDate] = useState(Date.now());

  const updateDuration = (e: ChangeEvent<HTMLInputElement>) => {
    setDuration(Number(e.target.value));
  };

  const startTimer = () => {
    setIsTimerActive(false);
    setCountDownDate(Date.now() + duration * 60 * 1000);
    setIsTimerActive(true);
    setRosExplorationTimer();
  };

  const stopTimer = () => {
    setIsTimerActive(false);
    setDuration(0);
    setRosExplorationTimer();
  };

  useEffect(() => {
    let interval: ReturnType<typeof setInterval> | undefined;
    const intervalMs = 1000;
    if (isTimerActive) {
      interval = setInterval(() => {
        setTimerDisplayProps(getTimeRemaining());
        setTimerDisplay(getTimeRemaining());
        setTimeRemaining(timeRemaining - intervalMs);
      }, intervalMs);
    } else if (!isTimerActive && timeRemaining !== 0) {
      if (interval !== undefined) {
        clearInterval(interval);
      }
      setTimerDisplay(timeDisplayDefault);
      setTimerDisplayProps(timeDisplayDefault);
      setTimerDisplay(timeDisplayDefault);
      setTimeRemaining(0);
    }

    const getTimeRemaining = () => {
      const total = countDownDate - Date.now();

      const minutes = Math.floor((total % (1000 * 60 * 60)) / (1000 * 60));
      const seconds = Math.floor((total % (1000 * 60)) / 1000);

      const minutesDiplay =
        minutes < 10 ? '0' + minutes.toString() : minutes.toString();
      const secondsDiplay =
        seconds < 10 ? '0' + seconds.toString() : seconds.toString();

      if (total < 0) {
        setIsTimerActive(false);
      }
      return `${minutesDiplay}:${secondsDiplay}`;
    };
    return () => clearInterval(interval);
  }, [isTimerActive, countDownDate, timeRemaining, setTimerDisplayProps]);

  const setRosExplorationTimer = () => {
    rosClient
      .callService(
        {
          name: `/start_exploration`,
        },
        { timeout: duration * 60 }
      )
      .catch(log.error);
  };

  return (
    <StyledDiv>
      <div>
        <LabeledInput
          label="Duration of exploration (min)"
          value={duration.toString()}
          type="number"
          onChange={updateDuration}
        />
        <StyledDivDuration>
          <Button
            onClick={startTimer}
            btnType="success"
            style={{ maxWidth: '185px' }}
          >
            Start
          </Button>
          <Button
            onClick={stopTimer}
            btnType="danger"
            style={{ maxWidth: '185px' }}
          >
            Stop
          </Button>
        </StyledDivDuration>
      </div>
      <StyledDivInfo>
        <p>Time left</p>
        <p>{timerDisplay}</p>
      </StyledDivInfo>
    </StyledDiv>
  );
};

const StyledDiv = styled.div`
  margin: 1em;
`;

const StyledDivDuration = styled.div`
  display: flex;
  column-gap: 20px;
`;

const StyledDivInfo = styled(StyledDivDuration)`
  margin: 8px;
  column-gap: 45px;
`;
