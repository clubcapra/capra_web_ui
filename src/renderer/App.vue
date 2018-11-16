<template>
  <div id="container">
    <navbar />
    <div id="view"><router-view /></div>
  </div>
</template>

<script>
import Navbar from '@/components/Navbar'
import { mapState, mapActions } from 'vuex'
import RosClient from './RosClient.js'

const rosClient = new RosClient()

export default {
  name: 'App',
  components: {
    Navbar
  },
  computed: {
    ...mapState('ROS', {
      robotIP: state => state.robotIP
    })
  },
  created() {
    rosClient.setListeners({
      onConnection: this.onConnection,
      onClose: this.onClose
    })
    rosClient.connect(this.robotIP)
  },
  provide: {
    rosClient
  },
  methods: {
    ...mapActions('ROS', {
      onConnection: 'connect',
      onClose: 'disconnect'
    })
  }
}
</script>

<style lang="scss">
html {
  height: 100%;
}

body {
  min-height: 100%;
  display: grid;
  align-content: stretch;
}

#container {
  display: grid;
  grid-template-rows: 60px auto 10px;

  #view {
    display: grid;
    align-content: stretch;
  }
}
</style>
