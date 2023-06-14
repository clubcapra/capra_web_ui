import React, { FC } from 'react';
import { styled } from '@/renderer/globalStyles/styled';
import { nanoid } from 'nanoid';
import { ReactNode } from 'react';
import { FaXbox } from 'react-icons/fa';

interface ShortcutData {
  content: ReactNode;
  from: [number, number];
  to: [number, number];
  key?: string;
}

const LEFT_X = 0;
const RIGHT_X = 800;

let data: ShortcutData[] = [
  // #region Sticks
  { content: 'Robot Direction', from: [LEFT_X, 250], to: [310, 250] }, // LS
  { content: 'Flipper Direction', from: [RIGHT_X, 340], to: [620, 340] }, // RS
  // #endregion

  // #region Triggers and bumpers
  { content: 'TPV Enabled', from: [LEFT_X, 160], to: [300, 160] }, // LB
  { content: 'Turbo Enabled', from: [RIGHT_X, 140], to: [680, 155] }, // RB
  // #endregion

  // #region DPAD
  {
    content: 'Current Mode Configuration',
    from: [LEFT_X, 320],
    to: [400, 320],
  },
  // //#endregion

  // #region XYAB
  { content: 'X: Stop Navigation', from: [RIGHT_X, 180], to: [610, 230] },
  { content: 'Y: Switch Front Back', from: [RIGHT_X, 211], to: [675, 211] },
  {
    content: 'B: Reset Flipper Position',
    from: [RIGHT_X, 250],
    to: [715, 250],
  },
  { content: 'A: Dead Man Switch', from: [RIGHT_X, 290], to: [675, 290] },
  // #endregion

  // Center
  {
    content: (
      <div>
        <FaXbox /> {'EStop'}
      </div>
    ),
    from: [500, 100],
    to: [500, 160],
  },
  { content: 'Select: ', from: [LEFT_X, 450], to: [450, 290] },
  { content: 'Start: Mode Switch', from: [RIGHT_X, 450], to: [550, 290] },
];

data = data.map((item) => {
  if (!item.key) {
    item.key = nanoid();
  }
  return item;
});

const GamepadComponent = () => {
  return (
    <StyledGamepad>
      {data.map((x) => (
        <GamepadBindingInfo key={x.key} from={x.from} to={x.to}>
          {x.content}
        </GamepadBindingInfo>
      ))}
      <img src="assets/images/xbox-one-controller.png" alt="xbox one gamepad" />
    </StyledGamepad>
  );
};

export const GamepadConfig = () => {
  return (
    <GamepadConfigWrapper>
      <GamepadComponent />
    </GamepadConfigWrapper>
  );
};

const GamepadBindingInfo: FC<{
  from: [number, number];
  to: [number, number];
}> = ({ from, to, children }) => {
  return (
    <>
      <StyledGamepadBindingInfo position={from}>
        {children}
      </StyledGamepadBindingInfo>
      <StyledSvg width="100%" height="100%">
        <line
          x1={from[0]}
          y1={from[1]}
          x2={to[0]}
          y2={from[1]}
          strokeWidth="1"
          stroke="white"
        />
        <line
          x1={to[0]}
          y1={from[1]}
          x2={to[0]}
          y2={to[1]}
          strokeWidth="1"
          stroke="white"
        />
      </StyledSvg>
    </>
  );
};

const StyledGamepadBindingInfo = styled.div<{
  position: [number, number];
}>`
  position: absolute;
  left: ${({ position }) => position[0]}px;
  top: ${({ position }) => position[1]}px;
  padding: 2px;
  border: 1px solid ${({ theme }) => theme.colors.border};
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  z-index: 3;
  display: table;
`;

const StyledSvg = styled.svg`
  position: absolute;
  z-index: 2;
  left: 1px;
  top: 1px;
`;

const GamepadConfigWrapper = styled.div`
  width: 100%;
  display: flex;
  justify-content: center;
  align-items: center;
  position: relative;
`;

const StyledGamepad = styled.div`
  padding-top: 40px;
  margin: 0 auto;
  position: relative;
  display: table;
  width: 1000px;

  img {
    position: relative;
    top: 0%;
    left: 50%;
    transform: translateX(-50%);
    width: 600px;
    height: 500px;
    z-index: 1;
  }
`;
