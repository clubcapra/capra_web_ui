<template>
  <div class="global-config">
    <ros />
    <hr />
    <teleop />
    <hr />
    <camera />
  </div>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

import Ros from './RosConfig.vue'
import Teleop from './TeleopConfig.vue'
import Camera from './CameraConfig.vue'

@Component({
  components: {
    Ros,
    Teleop,
    Camera,
  },
})
export default class GlobalConfig extends Vue {}
</script>

<style lang="scss" scoped>
.global-config {
  overflow-y: auto;
}
</style>
