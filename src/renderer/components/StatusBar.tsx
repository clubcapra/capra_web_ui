import { styled } from '@/renderer/globalStyles/styled';
import { useSelector } from '@/renderer/hooks/typedUseSelector';
import { useAudio } from '@/renderer/hooks/useAudio';
import { useInterval } from '@/renderer/hooks/useInterval';
import { useUpdateEffect } from '@/renderer/hooks/useUpdateEffect';
import { controlService } from '@/renderer/state/control';
import { flipperService } from '@/renderer/state/flipper';
import { rosService } from '@/renderer/state/ros';
import { feedSlice } from '@/renderer/store/modules/feed';
import { inputSlice, selectReverse } from '@/renderer/store/modules/input';
import { selectFullAddress } from '@/renderer/store/modules/ros';
import { useActor } from '@xstate/react';
import { format } from 'date-fns';
import { lighten } from 'polished';
import React, { FC, useState } from 'react';
import { BiWifi, BiWifi0, BiWifi1, BiWifi2, BiWifiOff } from 'react-icons/bi';
import { useDispatch } from 'react-redux';
import { toast } from 'react-toastify';
import { ArmContext, armService } from '../state/arm';
import { selectSelectedPreset } from '../store/modules/armPresets';

export const StatusBar: FC = () => (
  <StyledStatusBarWrapper>
    <LeftStatusBar>
      <RosConnectionStatus />
      <AudioStart />
      <Reverse />
      <Reload />
    </LeftStatusBar>
    <RightStatusBar>
      <ControlStatus />
      <ModeInfo />
      <NetworkInfo />
      <TimeDisplay />
    </RightStatusBar>
  </StyledStatusBarWrapper>
);

const RosConnectionStatus: FC = () => {
  const [state, send] = useActor(rosService);
  const fullAddress = useSelector(selectFullAddress);
  const onClick = () => {
    if (state.matches('disconnected')) {
      send('CONNECT');
    } else if (state.matches('connected')) {
      send('DISCONNECT');
    } else if (state.matches('connecting')) {
      // TODO send close
    }
  };

  return (
    <StatusBarButton onClick={onClick}>
      {state.matches('connected') && `Connected to ${fullAddress}`}
      {state.matches('connecting') && `Trying to connect to ${fullAddress}`}
      {state.matches('disconnected') && `Disconnected`}
    </StatusBarButton>
  );
};

const ControlStatus = () => {
  const [state] = useActor(controlService);
  return (
    <div>
      {state.matches('arm') && 'ARM'}
      {state.matches('flipper') && 'FLIPPER'}
      {state.matches('nothing') && 'NOTHING'}
    </div>
  );
};

const ModeInfo = () => {
  const [flipper] = useActor(flipperService);
  const [arm] = useActor(armService);
  const [control] = useActor(controlService);
  const isReverse = useSelector(selectReverse);
  const selectedArmPreset = useSelector(selectSelectedPreset);

  if (control.matches('flipper')) {
    return (
      <div>
        {flipper.matches('front') && (isReverse ? 'REAR' : 'FRONT')}
        {flipper.matches('fl') && (isReverse ? 'REAR RIGHT' : 'FRONT LEFT')}
        {flipper.matches('fr') && (isReverse ? 'REAR LEFT' : 'FRONT RIGHT')}
        {flipper.matches('rl') && (isReverse ? 'FRONT RIGHT' : 'REAR LEFT')}
        {flipper.matches('rr') && (isReverse ? 'FRONT LEFT' : 'REAR RIGHT')}
        {flipper.matches('all') && 'ALL'}
        {flipper.matches('rear') && (isReverse ? 'FRONT' : 'REAR')}
      </div>
    );
  } else if (control.matches('arm')) {
    return (
      <>
        <div>
          {arm.matches('joint')
            ? 'JOINT ' + String((arm.context as ArmContext).jointValue + 1)
            : 'CARTESIAN'}
        </div>
        <div>{selectedArmPreset.name.toUpperCase()}</div>
      </>
    );
  } else {
    return <div />;
  }
};

const NetworkInfo = () => {
  const [state, setState] = useState('offline');

  useInterval(() => {
    if (navigator) {
      type NetworkInformation = {
        downlink: number;
        rtt: number;
        effectiveType: string;
      };
      const { downlink, rtt, effectiveType } =
        navigator.connection as unknown as NetworkInformation;
      if (downlink == 0 && rtt == 0) {
        setState('offline');
      } else {
        setState(effectiveType);
      }
    }
  }, 500);

  const NetworkIcon = () => {
    switch (state) {
      case 'slow-2g':
        return <BiWifi0 />;
      case '2g':
        return <BiWifi1 />;
      case '3g':
        return <BiWifi2 />;
      case '4g':
        return <BiWifi />;
      default:
        return <BiWifiOff />;
    }
  };

  return (
    <div>
      <NetworkIcon />
    </div>
  );
};

const timeFormat = (date: Date) => format(date, 'HH:mm:ss');
const TimeDisplay: FC = () => {
  const [time, setTime] = useState(timeFormat(new Date()));

  useInterval(() => {
    setTime(timeFormat(new Date()));
  }, 1000);

  return <div>{time}</div>;
};

const AudioStart = () => {
  const [state] = useActor(rosService);
  const [isStarted, setIsStarted] = useState(false);
  const [start, stop] = useAudio();
  const onClick = () => {
    if (isStarted) {
      stop();
      setIsStarted(false);
    } else {
      start();
      setIsStarted(true);
    }
  };
  return (
    <StatusBarButton onClick={onClick} disabled={state.matches('disconnected')}>
      {isStarted ? 'Audio Stop' : 'Audio Start'}
    </StatusBarButton>
  );
};

const Reverse = () => {
  const reverse = useSelector(selectReverse);
  const dispatch = useDispatch();
  const toggleReverse = () => {
    // Maybe this should be combined somewhere in redux
    dispatch(feedSlice.actions.switchDirection());
    dispatch(inputSlice.actions.toggleReverse());
  };

  useUpdateEffect(() => {
    const id = toast.info(
      `Reverse mode toggled, forward direction is ${
        reverse ? 'flipped' : 'forward'
      }`
    );
    return () => {
      toast.dismiss(id);
    };
  }, [reverse]);

  return (
    <StatusBarButton onClick={toggleReverse}>
      {reverse ? 'Reverse' : 'Forward'}
    </StatusBarButton>
  );
};

const Reload = () => {
  return (
    <StatusBarButton
      onClick={() => {
        window.location.reload();
      }}
    >
      {'Reload'}
    </StatusBarButton>
  );
};

const StyledStatusBarWrapper = styled.div`
  display: grid;
  grid-template: 'l r';
  grid-template-columns: auto 1fr;
  height: 100%;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  color: ${({ theme }) => theme.colors.fontLight};
  box-shadow: 0 -1px 2px rgba(0, 0, 0, 0.5);
  font-size: 14px;
`;

const padding = 6;

const LeftStatusBar = styled.div`
  grid-area: l;
  display: flex;
  align-items: center;
  > * {
    padding: 0 ${padding}px;
  }
`;

const RightStatusBar = styled.div`
  grid-area: r;
  display: flex;
  justify-items: center;
  justify-content: flex-end;
  > * {
    padding: 0 ${padding}px;
  }
`;

const StatusBarButton = styled.button`
  font-family: inherit;
  margin: 0;
  border: 0;
  background-color: transparent;
  color: ${(ctx) => ctx.theme.colors.fontLight};
  &:hover {
    background-color: ${(ctx) => lighten(0.005)(ctx.theme.colors.background)};
  }
  &:disabled {
    cursor: not-allowed;
  }
`;
