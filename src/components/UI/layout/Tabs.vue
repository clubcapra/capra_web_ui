<template>
  <div class="custom-tabs">
    <b-tabs id="header">
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
    </b-tabs>
    <div class="logo">
      <img src="@/assets/logo.png" alt="capra logo" />
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'

@Component
export default class Tabs extends Vue {}
</script>

<style lang="scss" scoped>
.custom-tabs {
  display: grid;
  grid-template-columns: auto 120px;
  .tabs#header {
    margin-bottom: 0px;
  }
  .logo {
    border-bottom: 1px solid $text;
    padding-top: 3px;
    padding-right: 3px;
  }
}
</style>
