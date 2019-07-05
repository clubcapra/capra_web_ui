<template functional>
  <section class="config-menu">
    <aside class="menu">
      <b-menu-label>General</b-menu-label>
      <b-menu-list>
        <li>
          <router-link to="/configuration/ros" active-class="is-active"
            >ROS</router-link
          >
        </li>
        <li>
          <router-link to="/configuration/camera" active-class="is-active"
            >Camera</router-link
          >
        </li>
        <li>
          <router-link to="/configuration/gamepad" active-class="is-active"
            >Gamepad</router-link
          >
        </li>
      </b-menu-list>
      <b-menu-label>Tabs</b-menu-label>
      <b-menu-list>
        <li>
          <router-link to="/configuration/teleop" active-class="is-active"
            >Teleop</router-link
          >
        </li>
        <li>
          <router-link to="/configuration/victim" active-class="is-active"
            >Victim</router-link
          >
        </li>
      </b-menu-list>
    </aside>
  </section>
</template>

<style lang="scss" scoped>
$menu-padding: 12px;
.config-menu {
  border-right: 1px solid $text;
  padding-top: $menu-padding;
  padding-left: $menu-padding;
  padding-bottom: $menu-padding;
}
</style>
