import React, { FC, useRef } from 'react';
import { styled } from '@/renderer/globalStyles/styled';
import Nav2d from 'react-nav2djs';
import { rosClient } from '@/renderer/utils/ros/rosClient';
import { useRefSize } from '@/renderer/hooks/useRefSize';

const StyledViewer = styled.div`
  height: 100%;
  width: 100%;
  overflow: hidden;
`;

export const MapFeed: FC = () => {
  const ref = useRef<HTMLDivElement>(null);
  const [width, height] = useRefSize(ref);

  return (
    <StyledViewer ref={ref}>
      <Nav2d
        ros={rosClient.ros}
        id="nav2d"
        width={width}
        height={height}
        serverName={'/move_base'}
      />
    </StyledViewer>
  );
};
