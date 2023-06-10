import { Button } from '@/renderer/components/common/Button';
import { rosClient } from '@/renderer/utils/ros/rosClient';
import React, { useEffect, FC, useState, ChangeEvent } from 'react';
import { LabeledInput } from '@/renderer/components/common/LabeledInput';
import { log } from '@/renderer/logger';
import { styled } from '@/renderer/globalStyles/styled';

const StyledDiv = styled.div`
  display: flex;
  column-gap: 20px;
`;

const StyledDivInfo = styled.div`
  margin: 8px;
`;

export const ExplorationConfig: FC = () => {
  const [duration, setDuration] = useState(0);
  const [timeRemaining, setTimeRemaining] = useState(Number);
  const [isTimerActive, setIsTimerActive] = useState(false);
  const [timerDisplay, setTimerDisplay] = useState('00:00');
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
        setTimerDisplay(getTimeRemaining());
        setTimeRemaining(timeRemaining - intervalMs);
      }, intervalMs);
    } else if (!isTimerActive && timeRemaining !== 0) {
      if (interval !== undefined) {
        clearInterval(interval);
      }
      setTimerDisplay('00:00');
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
  }, [isTimerActive, countDownDate, timeRemaining]);

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
    <>
      <StyledDiv>
        <div>
          <LabeledInput
            label="Duration of exploration (min)"
            value={duration.toString()}
            type="number"
            onChange={updateDuration}
          />
          <StyledDiv>
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
          </StyledDiv>
        </div>
        <StyledDivInfo>
          <p>Time left</p>
          <p>{timerDisplay}</p>
        </StyledDivInfo>
      </StyledDiv>
    </>
  );
};
