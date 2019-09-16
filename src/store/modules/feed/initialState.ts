import { FeedState, FeedTypeEnum, CameraType } from 'store/modules/feed/@types'

export const initialState: FeedState = {
  feedMap: {},
  feeds: {
    minimap_2d: {
      type: FeedTypeEnum.minimap2D,
      id: 'minimap_2d',
    },
    minimap_3d: {
      type: FeedTypeEnum.minimap3D,
      id: 'minimap_3d',
    },
    model: {
      type: FeedTypeEnum.model,
      id: 'model',
    },
    joystick: {
      type: FeedTypeEnum.joystick,
      id: 'joystick',
    },
    camera_3d_rgb: {
      type: FeedTypeEnum.camera,
      id: 'camera_3d_rgb',
      camera: {
        name: 'camera_3d_rgb',
        type: CameraType.MJPEG,
        topic: '/camera_3d/rgb',
      },
    },
    webcam: {
      type: FeedTypeEnum.camera,
      id: 'webcam',
      camera: {
        name: 'webcam',
        type: CameraType.WEBCAM,
        topic: '',
      },
    },
  },
}
