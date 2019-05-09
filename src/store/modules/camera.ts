import { Module, VuexModule, Mutation } from 'vuex-module-decorators'
import { CameraMap, CameraType, Camera } from './camera.types'

@Module({ name: 'camera', namespaced: true })
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

  typesForSelect = [
    { disabled: false, value: CameraType.MJPEG },
    { disabled: false, value: CameraType.VP8 },
    { disabled: true, value: CameraType.WEB_RTC },
  ]

  @Mutation
  setTopic(payload: { cameraName: string; topic: string }) {
    this.cameras[payload.cameraName].topic = payload.topic
  }

  @Mutation
  setType(payload: { cameraName: string; type: CameraType }) {
    this.cameras[payload.cameraName].type = payload.type
  }

  @Mutation
  addCamera(payload: Camera) {
    const { type = CameraType.MJPEG, topic = '' } = payload.options
    this.cameras = { ...this.cameras, [payload.cameraName]: { type, topic } }
  }
}
