import React, { useCallback, useState } from 'react';
import {
  ArmPreset,
  armPresetsSlice,
} from '@/renderer/store/modules/armPresets';
import { styled } from '@/renderer/globalStyles/styled';
import { useDispatch } from 'react-redux';
import { FaCheck, FaPen, FaTimes } from 'react-icons/fa';
import ArmJointInput from './ArmJointInput';
import { Input } from '@/renderer/components/common/Input';
import { Modal } from '@/renderer/components/common/Modal';
import { useOpenClose } from '@/renderer/hooks/useOpenClose';
import { Button } from '@/renderer/components/common/Button';

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
  { min: 100, max: 260 }, //Joint 2
  { min: 95, max: 260 }, //Joint 3
  { min: 0, max: 360 }, //Joint 4
  { min: 100, max: 250 }, //Joint 5
  { min: 0, max: 360 }, //Joint 6
];

const Card = styled.div`
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  border-radius: 4px;
  padding: 16px;
  margin-top: 8px;
  margin-right: 8px;
  display: flex;
  width: 350px;
  flex-direction: column;
  justify-content: flex-start;
  align-items: flex-start;
  box-shadow: 0 0 5px 0 rgba(0, 0, 0, 0.5);
`;

const HeaderRow = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: space-between;
  align-items: center;
  width: 100%;
`;

const InputRow = styled.div`
  display: flex;
  flex-direction: row;
  width: 100%;
  justify-content: space-between;
  flex-wrap: wrap;
`;

const ButtonRow = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: flex-start;
  align-items: flex-start;
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

const EditButton = styled.a`
  border: none;
  border-radius: 4px;
  padding: 4px 8px;
  margin-top: 8px;
  cursor: pointer;
  transition: background-color 0.2s ease-in-out;
  &:hover {
    color: ${({ theme }) => theme.colors.success};
  }
`;

const ArmPreset = ({ preset }: ArmPresetProps) => {
  const dispatch = useDispatch();
  const [editMode, setEditMode] = useState(false);
  const [isModalOpen, openModal, closeModal] = useOpenClose();

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
    <>
      <Card>
        {editMode ? (
          <>
            <HeaderRow>
              <Input value={preset.name} type="text" onChange={onNameChange} />
              <EditButton onClick={() => setEditMode(!editMode)}>
                {<FaCheck />}
              </EditButton>
            </HeaderRow>
            <InputRow>
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
            </InputRow>
          </>
        ) : (
          <HeaderRow>
            <h3>{preset.name}</h3>
            <ButtonRow>
              <EditButton onClick={() => setEditMode(!editMode)}>
                {<FaPen />}
              </EditButton>
              {!editMode && (
                <DeleteButton onClick={openModal}>
                  <FaTimes />
                </DeleteButton>
              )}
            </ButtonRow>
          </HeaderRow>
        )}
      </Card>
      <Modal
        title="Do you really want to delete the preset"
        isOpen={isModalOpen}
        onClose={closeModal}
      >
        <ButtonRow>
          <Button onClick={() => onDelete(preset.id)} btnType="danger">
            Delete
          </Button>
          <Button onClick={closeModal}>Cancel</Button>
        </ButtonRow>
      </Modal>
    </>
  );
};

export default ArmPreset;
