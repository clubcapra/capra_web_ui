import React, { useEffect, useRef } from 'react';
import JSMpeg from '@cycjimmy/jsmpeg-player';
import { log } from '@/renderer/logger';

export const RTSPFeed = () => {
  // Execute rtspServer node script with stream url as argument
  const videoRef = useRef<HTMLDivElement>(null);
  useEffect(() => {
    const startServer = async () => {
      const port = (await window.preloadApi.rtsp.start('rtsp://')) as number;
      log.info(`RTSP server started on port ${port}`);
    };
    startServer().catch((e) => log.error(e));
  }, []);

  // Create video element
  useEffect(() => {
    const videoWrapper = videoRef.current;
    if (videoWrapper) {
      new JSMpeg.VideoElement(videoWrapper, 'ws://localhost:9999', {
        autoplay: true,
        audio: false,
      });
    }
  }, [videoRef]);

  return (
    <>
      <div ref={videoRef} />
    </>
  );
};
