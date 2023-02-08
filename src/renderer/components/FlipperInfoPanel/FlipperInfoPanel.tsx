import { styled } from '@/renderer/globalStyles/styled';
import { useFlippersMotorCurrents } from '@/renderer/hooks/useFlippersMotorCurrents';
import React, { memo, useEffect, useState } from 'react';
import { FlipperMotorCurrentInfo } from './FlipperMotorCurrentInfo';

const CURRENT_THRESHOLD = 0.3;
let currentTimeout: NodeJS.Timeout;

const StyledFlipperPanel = styled.div`
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  position: absolute;
  top: 10%;
  left: 50%;
  transform: translate(-50%, -50%);
  opacity: 1;
  background-color: ${({ theme }) => theme.colors.overlayBackground};
  transition: all 0.5s;
  z-index: 2;
  padding: 10px;
  border-radius: 10px;
  width: 320px;
`;

const StyledFlipperPanelRow = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: space-between;
  align-items: center;
  width: 100%;
  padding: 5px;
`;

const FlipperInfoPanel = () => {
  const {
    flMotorCurrentState,
    frMotorCurrentState,
    rlMotorCurrentState,
    rrMotorCurrentState,
  } = useFlippersMotorCurrents();

  const [isVisible, setVisible] = useState(false);

  useEffect(() => {
    // Clear previous timeout
    clearTimeout(currentTimeout);
    if (
      flMotorCurrentState > CURRENT_THRESHOLD ||
      frMotorCurrentState > CURRENT_THRESHOLD ||
      rlMotorCurrentState > CURRENT_THRESHOLD ||
      rrMotorCurrentState > CURRENT_THRESHOLD
    ) {
      setVisible(true);
    } else {
      currentTimeout = setTimeout(() => {
        setVisible(false);
      }, 2000);
    }
  }, [
    flMotorCurrentState,
    frMotorCurrentState,
    rlMotorCurrentState,
    rrMotorCurrentState,
  ]);

  return (
    <StyledFlipperPanel style={{ opacity: isVisible ? 1 : 0 }}>
      <StyledFlipperPanelRow>
        <FlipperMotorCurrentInfo
          name="Front left"
          value={flMotorCurrentState}
        />
        <FlipperMotorCurrentInfo
          name="Front right"
          value={frMotorCurrentState}
        />
      </StyledFlipperPanelRow>

      <StyledFlipperPanelRow>
        <FlipperMotorCurrentInfo name="Rear left" value={rlMotorCurrentState} />
        <FlipperMotorCurrentInfo
          name="Rear right"
          value={rrMotorCurrentState}
        />
      </StyledFlipperPanelRow>
    </StyledFlipperPanel>
  );
};

export default memo(FlipperInfoPanel);
