import { Button } from '@/renderer/components/common/Button';
import { Select } from '@/renderer/components/common/Select';
import { styled } from '@/renderer/globalStyles/styled';
import useArmJointPositions from '@/renderer/hooks/useArmJointPositions';
import {
  armPresetsSlice,
  selectAllPresets,
  selectSelectedPreset,
} from '@/renderer/store/modules/armPresets';
import { round } from 'lodash';
import { nanoid } from 'nanoid';
import React, { useCallback } from 'react';
import { FaPlus } from 'react-icons/fa';
import { useDispatch, useSelector } from 'react-redux';
import { SectionTitle } from '../../styles';
import ArmPreset from './ArmPreset';

const PresetsContainer = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: flex-start;
  align-items: flex-start;
  width: 100%;
  flex-wrap: wrap;
`;

const ArmPresetsConfig = () => {
  const armPresets = useSelector(selectAllPresets);
  const selectedPreset = useSelector(selectSelectedPreset);
  const dispatch = useDispatch();
  const jointPositions = useArmJointPositions() ?? [];

  const addPreset = useCallback(() => {
    dispatch(
      armPresetsSlice.actions.addPreset({
        id: nanoid(),
        name: 'New Preset',
        positions: [180, 180, 180, 180, 180, 180],
      })
    );
  }, [dispatch]);

  const onPresetSelect = useCallback(
    (e: React.ChangeEvent<HTMLSelectElement>) => {
      dispatch(armPresetsSlice.actions.selectPreset(e.target.value));
    },
    [dispatch]
  );

  return (
    armPresets &&
    selectedPreset && (
      <>
        <SectionTitle>Selected Preset</SectionTitle>
        <Select
          options={armPresets.map((preset) => ({
            key: preset.id,
            content: preset.name,
            value: preset.id,
          }))}
          value={selectedPreset.id}
          onChange={onPresetSelect}
        />
        {jointPositions.length > 0 && (
          <>
            <SectionTitle>Current positions</SectionTitle>
            <div>
              {jointPositions.map((position, index) => (
                <div key={index}>
                  Joint {index + 1}: {round(position, 0)}
                </div>
              ))}
            </div>
          </>
        )}

        <SectionTitle>Presets</SectionTitle>
        <Button onClick={addPreset}>
          <FaPlus />
          <span style={{ marginLeft: '0.2rem' }}>Add Preset</span>
        </Button>
        <PresetsContainer>
          {armPresets.map((preset) => (
            <ArmPreset key={preset.id} preset={preset} />
          ))}
        </PresetsContainer>
      </>
    )
  );
};

export default ArmPresetsConfig;
