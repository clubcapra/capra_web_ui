<template>
  <div id="header" class="tabs">
    <ul>
      <router-link active-class="is-active" tag="li" to="/teleop">
        <a>Teleop</a>
      </router-link>
      <router-link active-class="is-active" tag="li" to="/victim">
        <a>Victim</a>
      </router-link>
      <router-link active-class="is-active" tag="li" to="/settings">
        <a>Settings</a>
      </router-link>
    </ul>
  </div>
</template>

<script>
export default {
  name: 'Navbar'
}
</script>

<style lang="scss" scoped>
div.tabs {
  margin-bottom: 0px;
}
</style>
