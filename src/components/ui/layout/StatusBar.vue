<template>
  <footer class="takin-footer">
    <div>allo</div>
    <div>allo2</div>
    <div>allo2</div>
    <div>allo2</div>
    <div v-text="currentTime"></div>
  </footer>
</template>

<script>
import { Vue, Component } from 'vue-property-decorator'

@Component
export default class StatusBar extends Vue {
  currentTime = (new Date()).toLocaleTimeString()

  mounted(){
    this.interval = setInterval(this.updateTime, 1000);
  }

  updateTime() {
    this.currentTime = (new Date()).toLocaleTimeString()
  }

  destroyed() {
    clearInterval(this.interval)
  }

}
</script>

<style lang="scss">
$Padding-offset: 0.5%;

.takin-footer {
  //border-top: 1px solid black;
  height: 100%;
  font-size: 0.75em;
  background-color: purple;
  display: flex;
  flex-wrap: nowrap;
  justify-content: space-between;
  padding-left: $Padding-offset;
  padding-right: $Padding-offset;
}
</style>
