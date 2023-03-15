import { LabeledInput } from '@/renderer/components/common/LabeledInput';
import { defaultTheme } from '@/renderer/globalStyles/themes/defaultTheme';
import React, { useCallback, useState } from 'react';

interface ArmJointInputProps {
  onChange: (event: React.ChangeEvent<HTMLInputElement>, id: number) => void;
  value: number;
  id: number;
  min: number;
  max: number;
}

const ArmJointInput = ({
  onChange,
  value,
  id,
  min,
  max,
}: ArmJointInputProps) => {
  const [inputValue, setInputValue] = useState(value.toString());
  const [error, setError] = useState(false);

  const onInputChange = useCallback(
    (event: React.ChangeEvent<HTMLInputElement>) => {
      const jointValue = parseInt(event.target.value);
      setInputValue(event.target.value);
      if (jointValue >= min && jointValue <= max) {
        onChange(event, id);
        setError(false);
      } else {
        setError(true);
      }
    },
    [id, max, min, onChange]
  );

  return (
    <div>
      <LabeledInput
        label={`Joint ${id + 1}`}
        value={inputValue}
        type="number"
        onChange={onInputChange}
        labelStyle={
          error
            ? { color: defaultTheme.colors.danger }
            : { color: defaultTheme.colors.fontLight }
        }
      />
    </div>
  );
};

export default ArmJointInput;
