import { FeedState, FeedType, CameraType } from 'store/modules/feed/@types'

export const initialState: FeedState = {
  feedMap: {},
  feeds: {
    minimap_2d: {
      type: FeedType.minimap2D,
      id: 'minimap_2d',
      socketConnection: 'AHHHHH!!!', // This will probably be handled differently
    },
    minimap_3d: {
      type: FeedType.minimap3D,
      id: 'minimap_3d',
      socketConnection: 'AHHHHH!!! (but in 3d)', // This will probably be handled differently
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
      id: 'teleop_cam_1',
      camera: {
        name: 'teleop_1',
        type: CameraType.img,
      },
    },
  },
}
