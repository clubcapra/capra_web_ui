import { Button } from '@/renderer/components/common/Button';
import React, { FC } from 'react';

interface LaunchElementProps {
  name: string;
  launchFile: string;
  packageName: string;
  onClick: (fileName: string, packageName: string) => void;
  isLaunched?: boolean;
}

export const LaunchElement: FC<LaunchElementProps> = (
  props: LaunchElementProps
) => {
  return (
    <>
      <div style={{ display: 'flex', marginTop: 5 }}>
        <h3 style={{ flexGrow: 1, width: '75%' }}>{props.name}</h3>
        <Button
          onClick={() => props.onClick(props.launchFile, props.packageName)}
          btnType={props.isLaunched ? 'danger' : 'success'}
        >
          {props.isLaunched ? 'Kill' : 'Launch'}
        </Button>
      </div>
    </>
  );
};
