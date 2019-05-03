<template>
  <div id="header" class="tabs is-small">
    <ul>
      <router-link active-class="is-active" tag="li" to="/teleop">
        <a>Teleop</a>
      </router-link>
      <router-link active-class="is-active" tag="li" to="/victim">
        <a>Victim</a>
      </router-link>
      <router-link active-class="is-active" tag="li" to="/configuration">
        <a>Config</a>
      </router-link>
    </ul>
  </div>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

@Component
export default class Navbar extends Vue {}
</script>

<style lang="scss" scoped>
div.tabs {
  margin-bottom: 0px;
}
</style>
