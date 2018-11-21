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
    ...mapState('ros', {
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
    ...mapActions('ros', {
      onConnection: 'connect',
      onClose: 'disconnect'
    })
  }
}
</script>

<style lang="scss">
@import '~bulma/sass/utilities/_all';
@import '~bulmaswatch/slate/_variables.scss';
@import '~bulma';
@import '~bulmaswatch/slate/_overrides.scss';

html {
  height: 100%;
  overflow-y: auto;
}

body {
  min-height: 100%;
  display: grid;
  align-content: stretch;
}

#container {
  display: grid;
  grid-template-rows: auto 1fr;
  overflow: auto;

  #view {
    display: grid;
    align-content: stretch;
    overflow: auto;
  }
}
</style>
