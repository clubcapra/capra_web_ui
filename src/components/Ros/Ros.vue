<template>
  <div id="ros">
    <div>connected: {{ connected }}</div>
    <pre id="error">{{ error }}</pre>
  </div>
</template>

<script src="./Ros.js"/>
<style lang="stylus" scoped>
#error {
  color: red;
}
</style>
