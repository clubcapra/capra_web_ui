import React, { FC, useState } from 'react';
import { styled } from '@/renderer/globalStyles/styled';
import { ExplorationTimer } from './ExplorationTimer';
import {
  StyledPopup,
  StyledPopupContent,
  StyledPopupContainer,
} from '@/renderer/components/styles';
import { GoTelescope } from 'react-icons/go';

export const ExplorationStatus: FC = () => {
  const [timerDisplay, setTimerDisplay] = useState('00:00');

  const setTimerDisplayProps = (timerToDisplay: string) => {
    setTimerDisplay(timerToDisplay);
  };

  const isShowTimerDisplay = () => {
    return timerDisplay !== '00:00';
  };

  return (
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
        <ExplorationTimer setTimerDisplayProps={setTimerDisplayProps} />
      </StyledPopupContent>
    </StyledPopup>
  );
};

const StyledGoTelescope = styled(GoTelescope)`
  height: 1.25em;
  width: 1.25em;
  margin-left: 0.5em;
`;
