<template>
  <div class="teleop">
    <div class="main-view">
      <Camera path="front.rgb" />
      <Camera path="front.depth" />
    </div>
    <div class="bottom-panel">
      <Dashboard />
      <Map2D />
      <div>
        <img
          src="http://robotwebtools.org/images/demos/ros3djs-fetch-urdf.jpg"
        />
      </div>
    </div>
  </div>
</template>

<script>
import Dashboard from '@/components/Dashboard'
import Camera from '@/components/Camera'
import Map2D from '@/components/Map2D'

export default {
  name: 'Teleop',
  components: {
    Dashboard,
    Camera,
    Map2D
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
