import { FeedState, FeedType, CameraType } from 'store/modules/feed/@types'

export const initialState: FeedState = {
  feedMap: {},
  feeds: {
    minimap_2d: {
      type: FeedType.minimap2D,
      id: 'minimap_2d',
    },
    minimap_3d: {
      type: FeedType.minimap3D,
      id: 'minimap_3d',
    },
    model: {
      type: FeedType.model,
      id: 'model',
    },
    joystick: {
      type: FeedType.joystick,
      id: 'joystick',
    },
    teleop_cam_1: {
      type: FeedType.video,
      id: 'camera_3d_rgb',
      camera: {
        name: 'camera_3d_rgb',
        type: CameraType.img,
        topic: '/camera_3d/rgb',
      },
    },
  },
}
