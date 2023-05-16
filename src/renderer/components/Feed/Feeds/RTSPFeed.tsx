import React, { useEffect, useRef } from 'react';
import JSMpeg from '@cycjimmy/jsmpeg-player';
import { log } from '@/renderer/logger';

interface Props {
  url: string;
  flipped: boolean;
  rotated: boolean;
}

export const RTSPFeed = ({ url, flipped, rotated }: Props) => {
  const videoRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const videoWrapper = videoRef.current;
    const canvas = canvasRef.current;
    let videoElement: JSMpeg.VideoElement | null = null;
    let port: number | null = null;
    const startServer = async () => {
      port = (await window.preloadApi.rtsp.start(url)) as number;

      if (videoWrapper && canvas) {
        log.info(`RTSP server started on port ${port}`);
        videoElement = new JSMpeg.VideoElement(
          videoWrapper,
          `ws://localhost:${port}`,
          {
            canvas,
            autoplay: true,
            audio: false,
          },
          { videoBufferSize: 2048 * 2048 }
        );
      }
    };

    startServer().catch((e) => log.error(e));

    return () => {
      if (videoElement && port) {
        log.info(`Stopping RTSP server for ${url}`);
        window.preloadApi.rtsp.stop(port);
        videoElement.destroy();
      }
    };
  }, [url]);

  return (
    <div
      ref={videoRef}
      style={{
        height: '100%',
        objectFit: 'contain',
        overflow: 'hidden',
        transform: `${flipped ? 'scaleX(-1)' : ''} ${
          rotated ? 'rotate(180deg)' : ''
        }`,
      }}
    >
      <canvas
        ref={canvasRef}
        style={{
          height: '100%',
          objectFit: 'contain',
          overflow: 'hidden',
        }}
      />
    </div>
  );
};
