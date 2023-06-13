import { styled } from '@/renderer/globalStyles/styled';
import {
  StyledPopup,
  StyledPopupContent,
  StyledPopupContainer,
} from '@/renderer/components/styles';
import { IoMdStopwatch } from 'react-icons/io';
import { Button } from '@/renderer/components/common/Button';
import React, { useEffect, FC, useState, ChangeEvent } from 'react';
import { LabeledInput } from '@/renderer/components/common/LabeledInput';

export const CountdownStatus: FC = () => {
  const timeDisplayDefault = '00:00';

  const [duration, setDuration] = useState(35);
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
  };

  const stopTimer = () => {
    setIsTimerActive(false);
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

  return (
    <StyledDiv>
      <StyledPopup
        trigger={
          <StyledPopupContainer>
            {isShowTimerDisplay() && timerDisplay}
            <StyledIoMdStopwatch />
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
                label="Duration of scenario (min)"
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

const StyledIoMdStopwatch = styled(IoMdStopwatch)`
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