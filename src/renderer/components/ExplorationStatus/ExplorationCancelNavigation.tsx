import { rosClient } from '@/renderer/utils/ros/rosClient';
import React, { FC } from 'react';
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types';
import { styled } from '@/renderer/globalStyles/styled';
import { MdOutlineCancelScheduleSend } from 'react-icons/md';
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

  return <StyledImCancelCircle onClick={onClick} />;
};

export const StyledImCancelCircle = styled(MdOutlineCancelScheduleSend)`
  margin: 0.25em 0.25em 0 0;
  color: red;
  &:hover {
    cursor: pointer;
  }
`;
