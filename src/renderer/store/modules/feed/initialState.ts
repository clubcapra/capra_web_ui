import {
  FeedState,
  FeedTypeEnum,
  CameraType,
} from '@/renderer/store/modules/feed/@types'

export const initialState: FeedState = {
  feedMap: {},
  feeds: {
    empty: {
      type: FeedTypeEnum.empty,
      id: 'empty',
    },
    camera_3d_rgb: {
      type: FeedTypeEnum.camera,
      id: 'camera_3d_rgb',
      camera: {
        name: 'camera_3d_rgb',
        type: CameraType.MJPEG,
        topic: '/camera_3d/rgb/image_raw',
      },
    },
    camera_3d_depth: {
      type: FeedTypeEnum.camera,
      id: 'camera_3d_depth',
      camera: {
        name: 'camera_3d_depth',
        type: CameraType.MJPEG,
        topic: '/camera_3d/depth/image',
      },
    },
    urdf_viewer: {
      type: FeedTypeEnum.urdf,
      id: 'urdf_viewer',
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
