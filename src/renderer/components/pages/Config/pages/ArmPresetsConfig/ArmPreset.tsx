import React from 'react';
import {
  ArmPreset,
  armPresetsSlice,
} from '@/renderer/store/modules/armPresets';
import { styled } from '@/renderer/globalStyles/styled';
import { LabeledInput } from '@/renderer/components/common/LabeledInput';
import { useDispatch } from 'react-redux';
import { FaTimes } from 'react-icons/fa';

interface ArmPresetProps {
  preset: ArmPreset;
}

const Card = styled.div`
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  border-radius: 4px;
  padding: 16px;
  margin: 8px;
  display: flex;
  max-width: 300px;
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

  const onJointChange = (
    event: React.ChangeEvent<HTMLInputElement>,
    id: number
  ) => {
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
  };

  const onNameChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    // Update the preset in the store with the new name.
    dispatch(
      armPresetsSlice.actions.updatePreset({
        ...preset,
        name: event.target.value,
      })
    );
  };

  const onDelete = (id: string) => {
    // Delete the preset from the store.
    dispatch(armPresetsSlice.actions.removePreset(id));
  };

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
          <LabeledInput
            label={`Joint ${index + 1}`}
            value={joint?.toString() || ''}
            type="number"
            onChange={(e) => onJointChange(e, index)}
          />
        </div>
      ))}
    </Card>
  );
};

export default ArmPreset;
