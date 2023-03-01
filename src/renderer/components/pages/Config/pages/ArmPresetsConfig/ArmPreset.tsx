import React, { useCallback } from 'react';
import {
  ArmPreset,
  armPresetsSlice,
} from '@/renderer/store/modules/armPresets';
import { styled } from '@/renderer/globalStyles/styled';
import { LabeledInput } from '@/renderer/components/common/LabeledInput';
import { useDispatch } from 'react-redux';
import { FaTimes } from 'react-icons/fa';
import ArmJointInput from './ArmJointInput';

interface ArmPresetProps {
  preset: ArmPreset;
}

//Temporary UI limit for each joint value.
interface JointBoundaries {
  min: number;
  max: number;
}

const jointBoundaries: JointBoundaries[] = [
  { min: 0, max: 360 }, //Joint 1
  { min: 100, max: 250 }, //Joint 2
  { min: 100, max: 250 }, //Joint 3
  { min: 0, max: 360 }, //Joint 4
  { min: 100, max: 250 }, //Joint 5
  { min: 0, max: 360 }, //Joint 6
];

const Card = styled.div`
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  border-radius: 4px;
  padding: 16px;
  margin: 8px;
  display: flex;
  min-width: 300px;
  flex-direction: column;
  justify-content: flex-start;
  align-items: flex-start;
  box-shadow: 0 0 10px 0 rgba(0, 0, 0, 0.5);
  transition: box-shadow 0.2s ease-in-out;
`;

const HeaderRow = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: space-between;
  align-items: center;
  width: 100%;
`;

const DeleteButton = styled.a`
  border: none;
  border-radius: 4px;
  padding: 4px 8px;
  margin-top: 8px;
  cursor: pointer;
  transition: background-color 0.2s ease-in-out;
  &:hover {
    color: ${({ theme }) => theme.colors.danger};
  }
`;

const ArmPreset = ({ preset }: ArmPresetProps) => {
  const dispatch = useDispatch();

  const onJointChange = useCallback(
    (event: React.ChangeEvent<HTMLInputElement>, id: number) => {
      const jointValue = parseInt(event.target.value);
      // Update the preset in the store with the new joint value.
      dispatch(
        armPresetsSlice.actions.updatePreset({
          ...preset,
          positions: [
            ...preset.positions.slice(0, id),
            jointValue,
            ...preset.positions.slice(id + 1),
          ],
        })
      );
    },
    [dispatch, preset]
  );

  const onNameChange = useCallback(
    (event: React.ChangeEvent<HTMLInputElement>) => {
      // Update the preset in the store with the new name.
      dispatch(
        armPresetsSlice.actions.updatePreset({
          ...preset,
          name: event.target.value,
        })
      );
    },
    [dispatch, preset]
  );

  const onDelete = useCallback(
    (id: string) => {
      // Delete the preset from the store.
      dispatch(armPresetsSlice.actions.removePreset(id));
    },
    [dispatch]
  );

  return (
    <Card>
      <HeaderRow>
        <h3>{preset.name}</h3>
        <DeleteButton onClick={() => onDelete(preset.id)}>
          <FaTimes />
        </DeleteButton>
      </HeaderRow>
      <LabeledInput
        label="Preset Name"
        value={preset.name}
        type="text"
        onChange={onNameChange}
      />
      {preset.positions.map((joint, index) => (
        <div key={index}>
          <ArmJointInput
            value={joint}
            id={index}
            onChange={onJointChange}
            min={jointBoundaries[index].min}
            max={jointBoundaries[index].max}
          />
        </div>
      ))}
    </Card>
  );
};

export default ArmPreset;
