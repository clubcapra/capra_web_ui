<script>
import Dashboard from '@/components/Dashboard'
import Camera from '@/components/Camera'
import Map2D from '@/components/Map2D'
import { mapState } from 'vuex'

export default {
  name: 'Teleop',
  computed: {
    ...mapState('camera', {
      camera3drgb: state => ({
        type: state.cameras.camera3d_rgb.type,
        topic: state.cameras.camera3d_rgb.topic
      }),
      camera3ddepth: state => ({
        type: state.cameras.camera3d_depth.type,
        topic: state.cameras.camera3d_depth.topic
      })
    })
  },
  render() {
    const { camera3drgb, camera3ddepth } = this
    return (
      <div class="teleop">
        <div class="main-view">
          <Camera type={camera3drgb.type} topic={camera3drgb.topic} />
          <Camera type={camera3ddepth.type} topic={camera3ddepth.topic} />
        </div>
        <div class="bottom-panel">
          <Dashboard />
          <Map2D />
          <div>
            <img src="http://robotwebtools.org/images/demos/ros3djs-fetch-urdf.jpg" />
          </div>
        </div>
      </div>
    )
  }
}
</script>

<style lang="scss">
.teleop {
  display: grid;
  align-content: stretch;
  grid-template-rows: 70% 30%;

  .main-view {
    display: grid;
    grid-template-columns: 1fr 1fr;

    > div {
      box-shadow: inset 0 0 0 1px #000000;
    }
  }

  .bottom-panel {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;

    > div {
      box-shadow: inset 0 0 0 1px #000000;
    }
  }
}
</style>
