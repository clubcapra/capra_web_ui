import { styled } from '@/renderer/globalStyles/styled';
import {
  StyledPopup,
  StyledPopupContent,
  StyledPopupContainer,
} from '@/renderer/components/styles';
import { Button } from '@/renderer/components/common/Button';
import React, { useEffect, FC, useState, ChangeEvent } from 'react';
import { LabeledInput } from '@/renderer/components/common/LabeledInput';

interface CountdownProps {
  icon: JSX.Element;
  labelPopup: string;
  durationDefault: number;
  onStartClick?: (duration: number) => void;
  onStopClick?: () => void;
  sideElement?: JSX.Element;
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
  const timeDisplayDefault = '00:00';

  const [duration, setDuration] = useState(durationDefault);
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
    if (onStartClick !== undefined) {
      onStartClick(duration);
    }
  };

  const stopTimer = () => {
    setIsTimerActive(false);
    if (onStopClick !== undefined) {
      onStopClick();
    }
  };

  const isShowTimerDisplay = () => {
    return timerDisplay !== '00:00';
  };

  useEffect(() => {
    let interval: ReturnType<typeof setInterval> | undefined;
    const intervalMs = 1000;
    if (isTimerActive && !isNowStopCountdownTimer) {
      interval = setInterval(() => {
        setTimerDisplay(getTimeRemaining());
        setTimeRemaining(timeRemaining - intervalMs);
      }, intervalMs);
    } else if (
      isNowStopCountdownTimer ||
      (!isTimerActive && timeRemaining !== 0)
    ) {
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

      const minutesDiplay = minutes.toString().padStart(2, '0');
      const secondsDiplay = seconds.toString().padStart(2, '0');

      if (total < 0) {
        setIsTimerActive(false);
      }
      return `${minutesDiplay}:${secondsDiplay}`;
    };
    return () => clearInterval(interval);
  }, [isTimerActive, isNowStopCountdownTimer, countDownDate, timeRemaining]);

  return (
    <StyledDiv>
      {sideElement}
      <StyledPopup
        trigger={
          <StyledPopupContainer>
            {isShowTimerDisplay() && timerDisplay}
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
