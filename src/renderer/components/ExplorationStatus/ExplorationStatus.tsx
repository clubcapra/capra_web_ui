import React, { FC } from 'react';
import { styled } from '@/renderer/globalStyles/styled';
import { ExplorationTimer } from './ExplorationTimer';
import {
  StyledPopup,
  StyledPopupContent,
  StyledPopupContainer,
} from '@/renderer/components/styles';
import { GoTelescope } from 'react-icons/go';

export const ExplorationStatus: FC = () => {
  return (
    <StyledPopup
      trigger={
        <StyledPopupContainer>
          <StyledGoTelescope />
          <p>Exploration</p>
        </StyledPopupContainer>
      }
      on="click"
      position="bottom center"
      arrow={false}
      repositionOnResize={true}
    >
      <StyledPopupContent>
        <ExplorationTimer />
      </StyledPopupContent>
    </StyledPopup>
  );
};

const StyledGoTelescope = styled(GoTelescope)`
  height: 1.25em;
  width: 1.25em;
`;
