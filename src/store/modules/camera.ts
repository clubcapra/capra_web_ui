import { VuexModule, mutation, Module } from 'vuex-class-component'
import { CameraMap, CameraType, Camera, CameraOptions } from './camera.types'

@Module({ namespacedPath: 'camera/' })
export default class CameraModule extends VuexModule {
  videoServerIP = 'localhost:8080'

  cameras: CameraMap = {
    camera1: {
      type: CameraType.MJPEG,
      topic: '/capra/camera_3d/rgb/image_raw',
    },
    camera2: {
      type: CameraType.MJPEG,
      topic: '/capra/camera_3d/depth/image_raw',
    },
  }

  get camerasForSelect() {
    return Object.entries(this.cameras).map((entry) => {
      return { name: entry[0], ...entry[1] }
    })
  }

  typesForSelect = [
    { disabled: false, value: CameraType.MJPEG },
    { disabled: false, value: CameraType.VP8 },
    { disabled: true, value: CameraType.WEB_RTC },
  ]

  @mutation
  setTopic(payload: { cameraName: string; topic: string }) {
    this.cameras[payload.cameraName].topic = payload.topic
  }

  @mutation
  setType(payload: { cameraName: string; type: CameraType }) {
    this.cameras[payload.cameraName].type = payload.type
  }

  @mutation
  addCamera(payload: Camera) {
    const { type = CameraType.MJPEG, topic = '' } = payload.options
    this.cameras = { ...this.cameras, [payload.cameraName]: { type, topic } }
  }
}
