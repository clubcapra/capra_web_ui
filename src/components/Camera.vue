<script lang="tsx">
import { Vue, Component, Prop } from 'vue-property-decorator'

import { cameraModule, rosModule } from '@/store'
import { CameraType } from '@/store/modules/camera.types.ts'
import { VNode } from 'vue'

@Component
export default class Camera extends Vue {
  @Prop({ type: String, default: CameraType.MJPEG })
  readonly type!: CameraType

  @Prop({ default: '', required: true })
  readonly topic!: string

  get connected() {
    return rosModule.connected
  }

  get stream() {
    return (
      `http://${rosModule.robotIP}:${cameraModule.videoServerPort}/stream` +
      `?topic=${this.topic}` +
      `&type=${this.type}`
    )
  }

  render(): VNode {
    const NoVideo = (context: any) => (
      <div class="no-video">
        <p>{context.props.text}</p>
      </div>
    )

    const Stream = () => {
      switch (this.type) {
        case CameraType.MJPEG:
        case CameraType.PNG:
          return <img src={this.stream} />
        case CameraType.VP8:
          return <video src={this.stream} autoplay preload="none" />
        default:
          return <NoVideo text="invalid type" />
      }
    }

    return (
      <div class="camera">
        {this.connected ? <Stream /> : <NoVideo text="no video" />}
      </div>
    )
  }
}
</script>

<style lang="scss" scoped>
.camera {
  display: grid;

  > * {
    display: grid;
    align-items: center;
    justify-items: center;
    height: 100%;
    width: 100%;
  }
}
</style>
