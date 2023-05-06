import React from 'react';
import { SectionTitle } from '../../styles';
import { useDispatch, useSelector } from 'react-redux';
import { selectAllGpioPins } from '@/renderer/store/modules/gpioPins';

export const GpioPinsConfig = () => {
  const dispatch = useDispatch();
  const gpioPins = useSelector(selectAllGpioPins);
  return (
    <div>
      <SectionTitle>GPIO Config</SectionTitle>
    </div>
  );
};
