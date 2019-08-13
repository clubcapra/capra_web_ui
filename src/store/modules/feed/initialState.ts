import { FeedState, FeedType } from 'store/modules/feed/@types'
import v1 from 'assets/static/images/victim_1.png'
import v2 from 'assets/static/images/victim_2.jpg'
import v3 from 'assets/static/images/victim_3.jpg'
import v4 from 'assets/static/images/victim_4.jpg'
import t1 from 'assets/static/images/teleop_1.jpg'
import t2 from 'assets/static/images/teleop_2.jpg'
import mod from 'assets/static/images/fake_3d.png'

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
      socketConnection: mod, // There SHOULD be something, though what should it be, I cannot tell.
    },
    joystick: {
      type: FeedType.joystick,
      id: 'joystick',
    },
    teleop_cam_1: {
      type: FeedType.video,
      id: 'teleop_cam_1',
      camera: {
        fileType: 'img',
        name: 'teleop_1',
        topic: 'death star',
        socketConnection: t1,
      },
    },
    teleop_cam_2: {
      type: FeedType.video,
      id: 'teleop_cam_2',
      camera: {
        fileType: 'img',
        name: 'teleop_2',
        topic: 'Spartoh',
        socketConnection: t2,
      },
    },
    victim_cam_1: {
      type: FeedType.video,
      id: 'victim_cam_1',
      camera: {
        fileType: 'img',
        name: 'victim_1',
        topic: 'Spartoh',
        socketConnection: v1,
      },
    },
    victim_cam_2: {
      type: FeedType.video,
      id: 'victim_cam_2',
      camera: {
        fileType: 'img',
        name: 'victim_2',
        topic: 'Spartoh',
        socketConnection: v2,
      },
    },
    victim_cam_3: {
      type: FeedType.video,
      id: 'victim_cam_3',
      camera: {
        fileType: 'img',
        name: 'victim_3',
        topic: 'Spartoh',
        socketConnection: v3,
      },
    },
    victim_cam_4: {
      type: FeedType.video,
      id: 'victim_cam_4',
      camera: {
        fileType: 'img',
        name: 'victim_4',
        topic: 'Spartoh',
        socketConnection: v4,
      },
    },
  },
}
