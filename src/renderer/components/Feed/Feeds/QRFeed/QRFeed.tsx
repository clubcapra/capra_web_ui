import React, {
  FC,
  MutableRefObject,
  useCallback,
  useEffect,
  useState,
} from 'react';
import QRScanner from 'qr-scanner';
import QRScanRegion from './QRScanRegion';

export interface Point {
  x: number;
  y: number;
}

interface Props {
  imageRef: MutableRefObject<HTMLImageElement | null>;
}

export const QRFeed: FC<Props> = ({ children, imageRef }) => {
  const [qrCodeMessage, setMessage] = useState('');
  const [qrCodeCorners, setQrCodeCorners] = useState<Point[]>([]);

  const startScanRoutine = useCallback(() => {
    return new Promise((resolve) => {
      if (imageRef.current) {
        QRScanner.scanImage(imageRef.current, {
          returnDetailedScanResult: true,
        })
          .then((result) => {
            setMessage(result.data);
            setQrCodeCorners(result.cornerPoints);
          })
          .catch(() => {
            setQrCodeCorners([]);
          })
          .finally(() => {
            resolve('scanned');
          });
      }
    });
  }, [imageRef]);

  useEffect(() => {
    const interval = setInterval(async () => {
      await startScanRoutine();
    }, 500);

    return () => {
      clearInterval(interval);
    };
  }, [startScanRoutine]);

  return (
    <>
      {children}
      {imageRef && (
        <QRScanRegion
          points={qrCodeCorners}
          message={qrCodeMessage}
          imageWidth={imageRef.current?.width ?? 0}
          imageHeight={imageRef.current?.height ?? 0}
        />
      )}
    </>
  );
};
