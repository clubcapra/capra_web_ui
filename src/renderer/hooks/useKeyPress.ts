import { useEffect, useState } from 'react';

export const useKeyPressed = (targetKey: string) => {
  const [keyPressed, setKeyPressed] = useState<boolean>(false);
  useEffect(() => {
    const downHandler = ({ key }: KeyboardEvent) => {
      if (key === targetKey) {
        setKeyPressed(true);
      }
    };
    const upHandler = ({ key }: KeyboardEvent) => {
      if (key === targetKey) {
        setKeyPressed(false);
      }
    };
    window.addEventListener('keydown', downHandler);
    window.addEventListener('keyup', upHandler);
    return () => {
      window.removeEventListener('keydown', downHandler);
      window.removeEventListener('keyup', upHandler);
    };
  }, [targetKey]);
  return keyPressed;
};

export const useKeyPress = (targetKey: string, callback: () => void) => {
  const isKeyPressed = useKeyPressed(targetKey);
  useEffect(() => {
    if (isKeyPressed) {
      callback();
    }
  }, [callback, isKeyPressed]);
};
