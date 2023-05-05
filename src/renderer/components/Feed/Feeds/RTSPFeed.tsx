import React, { useEffect, useRef } from 'react';
import JSMpeg from '@cycjimmy/jsmpeg-player';
import { log } from '@/renderer/logger';

interface Props {
  url: string;
}

export const RTSPFeed = ({ url }: Props) => {
  const videoRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    log.info(`Starting RTSP server for ${url}`);
    const videoWrapper = videoRef.current;
    const canvas = canvasRef.current;
    let videoElement: JSMpeg.VideoElement | null = null;
    const startServer = async () => {
      const port = (await window.preloadApi.rtsp.start(url)) as number;
      log.info(`RTSP server started on port ${port}`);
      if (videoWrapper && canvas) {
        videoElement = new JSMpeg.VideoElement(
          videoWrapper,
          `ws://localhost:${port}`,
          {
            canvas,
            autoplay: true,
            audio: false,
          }
        );
      }
    };
    startServer().catch((e) => log.error(e));

    return () => {
      log.info('Stopping RTSP server');
      window.preloadApi.rtsp.stop(url);
      if (videoElement) {
        videoElement.destroy();
      }
    };
  }, [url]);

  return (
    <>
      <div ref={videoRef}>
        <canvas ref={canvasRef} />
      </div>
    </>
  );
};
