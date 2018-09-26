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
html, body
  height 100%
  overflow-y auto !important
  min-height 0

#container
  display grid
  grid-template-rows 50px 90% 10px
  height 100%
  min-height 0
</style>
