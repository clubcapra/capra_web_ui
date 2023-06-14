import { styled } from '@/renderer/globalStyles/styled';
import { IoMdStopwatch } from 'react-icons/io';
import { Countdown } from '@/renderer/components/common/Countdown';
import React, { FC } from 'react';

export const CountdownStatus: FC = () => {
  return (
    <Countdown
      icon={<StyledIoMdStopwatch />}
      labelPopup={'scenario'}
      durationDefault={35}
    />
  );
};

const StyledIoMdStopwatch = styled(IoMdStopwatch)`
  height: 1.25em;
  width: 1.25em;
`;
