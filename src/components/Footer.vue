<template>
  <footer id="footer">
    <button id="stpButton" @click="handleClick">Stop !</button>
  </footer>
</template>

<script>
export default {
  name: 'Footer',
  inject: ['rosClient'],
  methods: {
    handleClick: function(event) {
      this.rosClient.callService({ name: 'takin_estop', serviceType: '' }, '')
    },
  },
}
</script>

<style lang="scss">
#footer {
  height: 3rem !important;
  width: 100%;
  position: fixed;
  bottom: 0pt;
  left: 0pt;
  background: repeating-linear-gradient(
    45deg,
    #ffff00,
    #ffff00 10px,
    #000000 10px,
    #000000 20px
  );
}
#stpButton {
  font-size: 20px;
  background-color: #f44336;
  color: #ffffff;
  padding: 10px 10px;
  border-radius: 8px;
  text-align: center;
  margin: -50px -50px;
  position: relative;
  top: 30%;
  left: 95%;
}
</style>
