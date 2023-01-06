import { styled } from '@/renderer/globalStyles/styled';
import React, { FC } from 'react';

const StyledTextFeed = styled.div`
  width: 100%;
  height: 100%;
  background-color: black;
  text-align: center;
  display: flex;
  justify-content: center;
  align-items: center;
`;

export const TextFeed: FC<{ text: string }> = ({ text }) => (
  <StyledTextFeed>{text}</StyledTextFeed>
);
