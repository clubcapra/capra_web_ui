<template>
  <div class="wrapper">
    <config-menu />
    <div class="global-config">
      <router-view />
    </div>
  </div>
</template>

<script lang="ts">
import { Vue, Component } from 'vue-property-decorator'
import ConfigMenu from './ConfigMenu.vue'

@Component({ components: { ConfigMenu } })
export default class GlobalConfig extends Vue {
  created() {
    this.$router.push('/configuration/ros')
  }
}
</script>

<style lang="scss" scoped>
.wrapper {
  display: grid;
  grid-template-columns: 250px auto;

  .global-config {
    overflow-y: auto;
  }
}
</style>
