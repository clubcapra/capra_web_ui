import {
  FeedState,
  FeedTypeEnum,
  CameraType,
} from '@/renderer/store/modules/feed/@types'

export const initialState: FeedState = {
  feedMap: {},
  feeds: {
    empty: {
      type: FeedTypeEnum.Empty,
      id: 'empty',
    },
    camera_3d_rgb: {
      type: FeedTypeEnum.Camera,
      id: 'camera_3d_rgb',
      camera: {
        name: 'camera_3d_rgb',
        type: CameraType.MJPEG,
        topic: '/camera_3d/rgb/image_raw',
      },
    },
    camera_3d_depth: {
      type: FeedTypeEnum.Camera,
      id: 'camera_3d_depth',
      camera: {
        name: 'camera_3d_depth',
        type: CameraType.MJPEG,
        topic: '/camera_3d/depth/image',
      },
    },
    webcam: {
      type: FeedTypeEnum.Camera,
      id: 'webcam',
      camera: {
        name: 'webcam',
        type: CameraType.WEBCAM,
        topic: '',
      },
    },
    urdf_viewer: {
      type: FeedTypeEnum.Urdf,
      id: 'urdf_viewer',
    },
    co2_graph: {
      type: FeedTypeEnum.Graph,
      id: 'co2_graph',
    },
  },
}
