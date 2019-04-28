import { Module, VuexModule, Mutation, Action } from 'vuex-module-decorators'
import store from '..'

interface CameraMap {
  [cameraName: string]: CameraOptions
}

type Camera = { cameraName: string; options: CameraOptions }

type CameraOptions = { type: string; topic: string }

@Module({ dynamic: true, store, name: 'camera', namespaced: true })
export default class CameraModule extends VuexModule {
  videoServerIP = 'localhost:8080'

  cameras: CameraMap = {
    camera3d_rgb: {
      type: 'mjpeg',
      topic: '/capra/camera_3d/rgb/image_raw',
    },
    camera3d_depth: {
      type: 'mjpeg',
      topic: '/capra/camera_3d/depth/image_raw',
    },
  }

  @Mutation
  setTopic(payload: { cameraName: string; topic: string }) {
    this.cameras[payload.cameraName].topic = payload.topic
  }

  @Mutation
  setType(payload: { cameraName: string; type: string }) {
    this.cameras[payload.cameraName].type = payload.type
  }

  @Mutation
  addCamera(payload: Camera) {
    const { type = 'mjpeg', topic = '' } = payload.options
    this.cameras = { ...this.cameras, [payload.cameraName]: { type, topic } }
  }
}
