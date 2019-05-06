<template>
  <div :id="id" ref="viewer" class="viewer-3d" />
</template>

<script lang="ts">
import { Vue, Component, Prop, Inject } from 'vue-property-decorator'

import { Viewer, Grid, UrdfClient, COLLADA_LOADER, MeshResource } from 'ros3d'
import { TFClient } from 'roslib'
import RosClient from '@/utils/ros//RosClient'

import _ from 'lodash-es'

@Component
export default class Viewer3D extends Vue {
  @Inject('rosClient') rosClient!: RosClient

  get id() {
    return _.uniqueId('ros-viewer-3d-')
  }

  mounted() {
    const refViewer = this.$refs.viewer as Element

    const mesh = new MeshResource({
      resource: 'test.dae',
      path: '/',
      warnings: true,
    })

    const viewer = new Viewer({
      divID: refViewer.id,
      width: refViewer.clientWidth - 1,
      height: refViewer.clientHeight - 1,
      antialias: true,
    })

    viewer.addObject(new Grid())
    viewer.addObject(mesh)

    const tfClient = new TFClient({
      ros: this.rosClient.ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
    })

    const urdfClient = new UrdfClient({
      ros: this.rosClient.ros,
      tfClient: tfClient,
      path: 'https://raw.githubusercontent.com/PR2/pr2_common/kinetic-devel/',
      rootObject: viewer.scene,
      loader: COLLADA_LOADER,
    })
  }
}
</script>

<style lang="scss" scoped>
.viewer-3d {
  overflow: hidden;
}
</style>
