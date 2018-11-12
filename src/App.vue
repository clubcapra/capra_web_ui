<template>
  <div id="container">
    <div
      id="header"
      class="tabs">
      <ul>
        <router-link
          active-class="is-active"
          tag="li"
          to="/teleop">
          <a>Teleop</a>
        </router-link>
        <router-link
          active-class="is-active"
          tag="li"
          to="/victim">
          <a>Victim</a>
        </router-link>
        <router-link
          active-class="is-active"
          tag="li"
          to="/settings">
          <a>Settings</a>
        </router-link>
      </ul>
    </div>
    <div id="view">
      <router-view />
    </div>
    <div id="footer">
      <ros />
    </div>
  </div>
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
html
  height 100%

body
  min-height 100%
  display grid
  align-content stretch

#container
  display grid
  grid-template-rows 60px auto 10px

  #view
    display grid
    align-content stretch
</style>
