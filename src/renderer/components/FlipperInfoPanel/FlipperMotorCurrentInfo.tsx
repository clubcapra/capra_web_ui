import { styled } from '@/renderer/globalStyles/styled';
import React from 'react';

interface Props {
  name: string;
  value: number;
}

const StyledCurrentValue = styled.p`
  transition: all 0.5s;
`;

const StyledDiv = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: space-between;
  align-items: center;
  padding: 5px;
  width: 50%;
`;

const StyledRow = styled.div`
  display: flex;
  flex-direction: row;
`;

const DANGER_THRESHOLD = 3;

export const FlipperMotorCurrentInfo = ({ name, value }: Props) => {
  return (
    <StyledDiv>
      <p>{name}</p>
      <StyledRow>
        <StyledCurrentValue
          style={{
            color: value > DANGER_THRESHOLD ? 'red' : 'green',
            marginRight: '5px',
          }}
        >
          {value}
        </StyledCurrentValue>
        <p>A</p>
      </StyledRow>
    </StyledDiv>
  );
};
