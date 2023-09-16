import React, { memo } from 'react';
import { StyledPopup } from '@/renderer/components/styles';
import { useSelector } from 'react-redux';
import { selectAllGpioPins } from '@/renderer/store/modules/gpioPins';
import { GpioPin } from './GpioPin';
import { styled } from '@/renderer/globalStyles/styled';
import { BsLightbulb } from 'react-icons/bs';
import { BsLightbulbOff } from 'react-icons/bs';

const GpioPinsStatus = () => {
  const gpioPins = useSelector(selectAllGpioPins);
  const isAGpioPinOn = gpioPins.find((gpioPin) => gpioPin.isOn)?.isOn;
  return (
    <StyledPopup
      trigger={
        <Container>
          {isAGpioPinOn ? <StyledBsLightbulb /> : <StyledBsLightbulbOff />}
        </Container>
      }
      on="click"
      position="bottom center"
      arrow={false}
      repositionOnResize={true}
    >
      <StyledDiv>
        {gpioPins.map((gpioPin) => (
          <GpioPin key={gpioPin.id} gpioPin={gpioPin} />
        ))}
      </StyledDiv>
    </StyledPopup>
  );
};

const Container = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: center;
  align-items: center;
  &:hover {
    cursor: pointer;
  }
`;

const StyledDiv = styled.div`
  display: flex;
  flex-direction: row;
  align-items: flex-start;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  box-shadow: 0 0 10px 0 rgba(0, 0, 0, 0.5);
  border-radius: 4px;
  min-width: 150px;
`;

const StyledBsLightbulb = styled(BsLightbulb)`
  height: 1.25em;
  width: 1.25em;
`;

const StyledBsLightbulbOff = styled(BsLightbulbOff)`
  height: 1.25em;
  width: 1.25em;
`;

export default memo(GpioPinsStatus);
