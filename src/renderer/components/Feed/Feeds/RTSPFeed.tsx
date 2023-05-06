import React, { useEffect, useRef, useState } from 'react';
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
  const [port, setPort] = useState<number | null>(null);
  const [videoElement, setVideoElement] =
    useState<JSMpeg.VideoElement | null>();

  useEffect(() => {
    const videoWrapper = videoRef.current;
    const canvas = canvasRef.current;
    const startServer = async () => {
      const port = (await window.preloadApi.rtsp.start(url)) as number;
      log.info(`RTSP server started on port ${port}`);
      if (videoWrapper && canvas) {
        setVideoElement(
          new JSMpeg.VideoElement(
            videoWrapper,
            `ws://localhost:${port}`,
            {
              canvas,
              autoplay: true,
              audio: false,
            },
            { videoBufferSize: 2048 * 2048 }
          )
        );
        setPort(port);
      }
    };
    if (!port && !videoElement) {
      startServer().catch((e) => log.error(e));
    }

    return () => {
      if (port) {
        log.info(`Stopping RTSP server for ${url}`);
        window.preloadApi.rtsp.stop(port);
        setPort(null);
        if (videoElement) {
          videoElement.destroy();
        }
      }
    };
  }, [port, url, videoElement]);

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
