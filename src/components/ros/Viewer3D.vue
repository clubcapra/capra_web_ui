<template>
  <div :id="id" ref="viewer" class="viewer-3d" />
</template>

<script lang="ts">
import { Vue, Component, Prop, Inject } from 'vue-property-decorator'

import { Viewer, Grid, UrdfClient, COLLADA_LOADER, MeshResource } from 'ros3d'

import { TFClient } from 'roslib'
import RosClient from '@/utils/ros/RosClient'

import _ from 'lodash-es'

@Component
export default class Viewer3D extends Vue {
  get id() {
    return _.uniqueId('ros-viewer-3d-')
  }

  initViewer(): Viewer {
    const refViewer = this.$refs.viewer as Element

    const viewer = new Viewer({
      divID: refViewer.id,
      width: refViewer.clientWidth - 1,
      height: refViewer.clientHeight - 1,
      antialias: true,
    })

    viewer.addObject(new Grid())

    window.addEventListener('resize', () => {
      viewer.resize(refViewer.clientWidth - 1, refViewer.clientHeight - 1)
    })

    return viewer
  }

  mounted() {
    // const mesh = new MeshResource({
    //   resource: 'test.dae',
    //   path: '/',
    //   warnings: true,
    // })
    // const viewer = this.initViewer()
    // viewer.addObject(mesh)
    // const tfClient = new TFClient({
    //   ros: this.rosClient.ros,
    //   angularThres: 0.01,
    //   transThres: 0.01,
    //   rate: 10.0,
    // })
    // const urdfClient = new UrdfClient({
    //   ros: this.rosClient.ros,
    //   tfClient: tfClient,
    //   rootObject: viewer.scene,
    //   loader: COLLADA_LOADER,
    // })
  }
}
</script>

<style lang="scss" scoped>
.viewer-3d {
  overflow: hidden;
}
</style>
