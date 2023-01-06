import { useEffect } from 'react';
import { toast } from 'react-toastify';

export const useAudio = () => {
  useEffect(() => {
    window.preloadApi.audio.receive(({ error }) => {
      if (error) {
        toast.error(error);
      }
    });
  }, []);
  const start = () => {
    window.preloadApi.audio.start();
  };
  const stop = () => {
    window.preloadApi.audio.stop();
  };
  return [start, stop];
};
