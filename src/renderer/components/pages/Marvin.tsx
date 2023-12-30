import React, { FC } from 'react';

export const Marvin: FC = () => {
  return (
      <iframe
        src="http://127.0.0.1:7860/"
        title="Example Site"
        style={{ width: '100%', height: '100%' }}
        allow="microphone"
      />
  );
};
