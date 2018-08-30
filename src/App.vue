<template lang="pug">
#app
  .row#header.tabs
    ul
      router-link(active-class='is-active' tag='li' to='/teleop')
        a Teleop
      router-link(active-class='is-active' tag='li' to='/victim')
        a Victim
      router-link(active-class='is-active' tag='li' to='/settings')
        a Settings
  .row#view
    router-view
  .row#footer
    ros
</template>

<script>
import Ros from '@/components/Ros'

export default {
  name: 'App',
  components: {
    Ros
  }
}
</script>

<style lang="stylus">
html, body
  height 100vh
  margin 0

#app
  display flex
  flex-flow column
  height 100%

#app .row#header
  flex 0 1 auto

#app .row#view
  flex 1 1 auto

#app .row#footer
  flex 0 1 10px
</style>
