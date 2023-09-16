import { styled } from '@/renderer/globalStyles/styled';
import {
  StyledPopup,
  StyledPopupContent,
  StyledPopupContainer,
} from '@/renderer/components/styles';
import { Button } from '@/renderer/components/common/Button';
import React, { useEffect, FC, useState, ChangeEvent, useRef } from 'react';
import { LabeledInput } from '@/renderer/components/common/LabeledInput';

const INTERVAL_MS = 1000;

interface CountdownProps {
  icon: JSX.Element;
  labelPopup: string;
  durationDefault: number;
  onStartClick?: (duration: number) => void;
  onStopClick?: () => void;
  sideElement?: JSX.Element;
  /**
   * If true, the timer will be stopped.
   */
  isNowStopCountdownTimer?: boolean;
}

export const Countdown: FC<CountdownProps> = ({
  icon,
  labelPopup,
  durationDefault,
  onStartClick,
  onStopClick,
  sideElement,
  isNowStopCountdownTimer,
}) => {
  const countDownDate = useRef<number>(0);
  const intervalRef = useRef<ReturnType<typeof setInterval>>();

  const [duration, setDuration] = useState(durationDefault);
  const [timerDisplay, setTimerDisplay] = useState('00:00');

  const getTimeRemaining = () => {
    return countDownDate.current - Date.now();
  };

  useEffect(() => {
    if (isNowStopCountdownTimer) {
      stopTimer();
      updateTimerDisplay();
    }
  }, [isNowStopCountdownTimer]);

  /**
   * Parses a time in milliseconds to a string in the format mm:ss
   *
   * @param timeRemain - time in milliseconds
   */
  const formatTime = (timeRemain: number): string => {
    const minutes = Math.floor((timeRemain % (1000 * 60 * 60)) / (1000 * 60));
    const seconds = Math.floor((timeRemain % (1000 * 60)) / 1000);

    const minutesDiplay = minutes.toString().padStart(2, '0');
    const secondsDiplay = seconds.toString().padStart(2, '0');

    return `${minutesDiplay}:${secondsDiplay}`;
  };

  const startTimer = () => {
    if (intervalRef.current !== undefined) {
      stopTimer();
    }
    countDownDate.current = Date.now() + duration * 60 * 1000;
    intervalRef.current = setInterval(handleTimerTick, INTERVAL_MS);
    if (onStartClick !== undefined) {
      onStartClick(duration);
    }
  };

  const stopTimer = () => {
    clearInterval(intervalRef.current);
    intervalRef.current = undefined;
    countDownDate.current = 0;
    if (onStopClick !== undefined) {
      onStopClick();
    }
  };

  /**
   * Updates the timer display.
   * If the timer is negative, it will be set to 0.
   */
  const updateTimerDisplay = () => {
    let time = getTimeRemaining();
    if (time < 0) {
      time = 0;
    }
    const timeDisplay = formatTime(time);
    setTimerDisplay(timeDisplay);
  };

  const isTimerActive = () => {
    return countDownDate.current > Date.now();
  };

  /**
   * This function is called when the user clicks the start button.
   */
  const handleStartButtonClick = () => {
    startTimer();
    handleTimerTick();
  };

  /**
   * This function is called when the user clicks the stop button.
   */
  const handleStopButtonClick = () => {
    stopTimer();
    updateTimerDisplay();
  };

  /**
   * This function is called when the user changes the duration.
   * @param e - the event
   */
  const handleDurationChange = (e: ChangeEvent<HTMLInputElement>) => {
    setDuration(Number(e.target.value));
  };

  /**
   * This function is called every second by the timer.
   */
  const handleTimerTick = () => {
    const time = getTimeRemaining();
    if (time <= 0) {
      countDownDate.current = 0;
      stopTimer();
    }

    updateTimerDisplay();
  };

  return (
    <StyledDiv>
      {sideElement}
      <StyledPopup
        trigger={
          <StyledPopupContainer>
            {isTimerActive() && timerDisplay}
            {icon}
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
                label={`Duration of ${labelPopup} (min)`}
                value={duration.toString()}
                type="number"
                onChange={handleDurationChange}
              />
              <StyledDivDuration>
                <Button
                  onClick={handleStartButtonClick}
                  btnType="success"
                  style={{ maxWidth: '185px' }}
                >
                  Start
                </Button>
                <Button
                  onClick={handleStopButtonClick}
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
              {sideElement}
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

const StyledDivTimer = styled.div`
  margin: 1em;
`;

const StyledDivDuration = styled.div`
  display: flex;
  column-gap: 20px;
`;

const StyledDivInfo = styled(StyledDivDuration)`
  margin: 8px;
`;

const StyledP = styled.p`
  margin-right: 20px;
`;
