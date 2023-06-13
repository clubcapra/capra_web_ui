import { rosClient } from '@/renderer/utils/ros/rosClient';
import React, { FC } from 'react';
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types';
import { styled } from '@/renderer/globalStyles/styled';
import { FaStop } from 'react-icons/fa';
import { toast } from 'react-toastify';

const stopNavigationTopic: TopicOptions = {
  name: '/move_base/cancel',
  messageType: 'actionlib_msgs/GoalID',
};

interface CancelNavigation {
  isCancelNavigationProps: () => void;
}

export const ExplorationCancelNavigation: FC<CancelNavigation> = ({
  isCancelNavigationProps,
}) => {
  const onClick = () => {
    rosClient.publish(stopNavigationTopic, {});
    isCancelNavigationProps();
    toast.info('Navigation stop');
  };

  return <StyledFaStop onClick={onClick} />;
};

export const StyledFaStop = styled(FaStop)`
  margin-top: 0.25em;
  color: red;
  &:hover {
    cursor: pointer;
  }
`;
