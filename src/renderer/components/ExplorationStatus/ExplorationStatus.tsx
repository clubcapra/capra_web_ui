import { styled } from '@/renderer/globalStyles/styled';
import { ExplorationCancelNavigation } from './ExplorationCancelNavigation';
import {
  StyledPopup,
  StyledPopupContent,
  StyledPopupContainer,
} from '@/renderer/components/styles';
import { GoTelescope } from 'react-icons/go';
import { Button } from '@/renderer/components/common/Button';
import { rosClient } from '@/renderer/utils/ros/rosClient';
import React, { useEffect, FC, useState, ChangeEvent } from 'react';
import { LabeledInput } from '@/renderer/components/common/LabeledInput';
import { log } from '@/renderer/logger';

export const ExplorationStatus: FC = () => {
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
    startRosExplorationTimer();
  };

  const stopTimer = () => {
    setIsTimerActive(false);
    stopRosExplorationTimer();
  };

  const isShowTimerDisplay = () => {
    return timerDisplay !== '00:00';
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
      setTimerDisplay(timeDisplayDefault);
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
  }, [isTimerActive, countDownDate, timeRemaining]);

  const startRosExplorationTimer = () => {
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
    <StyledDiv>
      <ExplorationCancelNavigation isCancelNavigationProps={stopTimer} />
      <StyledPopup
        trigger={
          <StyledPopupContainer>
            {isShowTimerDisplay() && timerDisplay}
            <StyledGoTelescope />
          </StyledPopupContainer>
        }
        on="click"
        position="bottom center"
        arrow={false}
        repositionOnResize={true}
      >
        <StyledPopupContent>
          <StyledDivTimer>
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
              <StyledP>Time left</StyledP>
              <p>{timerDisplay}</p>
              <ExplorationCancelNavigation
                isCancelNavigationProps={stopTimer}
              />
            </StyledDivInfo>
          </StyledDivTimer>
        </StyledPopupContent>
      </StyledPopup>
    </StyledDiv>
  );
};

const StyledDiv = styled.div`
  display: flex;
  column-gap: 5px;
`;

const StyledGoTelescope = styled(GoTelescope)`
  height: 1.25em;
  width: 1.25em;
`;
const StyledDivTimer = styled.div`
  margin: 1em;
`;

const StyledDivDuration = styled.div`
  display: flex;
  column-gap: 20px;
`;

const StyledP = styled.p`
  margin-right: 20px;
`;

const StyledDivInfo = styled(StyledDivDuration)`
  margin: 8px;
`;
