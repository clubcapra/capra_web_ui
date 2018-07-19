<template>
  <div id="app">
    <Ros />
    <div id="nav">
      <router-link to="/teleop">Teleop</router-link> |
      <router-link to="/victim">Victim</router-link>
    </div>
    <router-view/>
  </div>
</template>

<script src="./App.js" />

<style lang="stylus">
#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  color: #2c3e50;
}

#nav {
  padding: 30px;

  a {
    font-weight: bold;
    color: #2c3e50;

    &.router-link-exact-active {
      color: #42b983;
    }
  }
}
</style>
