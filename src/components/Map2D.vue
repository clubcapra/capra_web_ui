<template>
  <div id="mad2d" />
</template>

<script>
import RosClient from '@/lib/RosClient'

export default {
  name: 'Map2D',
  mounted() {
    let rosclient = new RosClient()
    rosclient.initMap2D('map2d')
  }
}
</script>
