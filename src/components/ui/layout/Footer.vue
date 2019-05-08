<template>
  <footer class="takin-footer">
    <div class="left" />
    <div class="right"><e-stop /></div>
  </footer>
</template>

<script>
import { Vue, Component } from 'vue-property-decorator'

@Component
export default class Footer extends Vue {}
</script>

<style lang="scss">
.takin-footer {
  display: grid;
  overflow: hidden;
  grid-template-columns: auto 65px;
  grid-template-areas: 'left right';

  .left {
    grid-area: left;
  }

  .right {
    grid-area: right;
  }

  background: repeating-linear-gradient(
    45deg,
    #ffff00,
    #ffff00 10px,
    #000000 10px,
    #000000 20px
  );
}
</style>
