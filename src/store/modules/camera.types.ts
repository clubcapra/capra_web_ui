export interface CameraMap {
  [cameraName: string]: CameraOptions
}

export interface Camera {
  cameraName: string
  options: CameraOptions
}

interface CameraOptions {
  type: CameraType
  topic: string
}

export enum CameraType {
  MJPEG = 'mjpeg',
  VP8 = 'vp8',
  WEB_RTC = 'web_rtc',
}
