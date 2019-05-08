<template>
  <footer class="takin-footer">footer</footer>
</template>

<script>
import { Vue, Component } from 'vue-property-decorator'

@Component
export default class Footer extends Vue {}
</script>

<style lang="scss">
.takin-footer {
  border-top: 1px solid black;
  height: 100%;
  font-size: 0.75em;
}
</style>
