import Vue from 'vue'
import vueBulmaComponents from 'vue-bulma-components'
import './fontAwesome'

import App from './App.vue'
import router from '@/router'
import store from '@/store'
import { gamepadManagerInstance } from '@/utils/gamepad/GamepadManager'

gamepadManagerInstance.start()

Vue.config.productionTip = false

Vue.use(vueBulmaComponents)

new Vue({
  router,
  store,
  render: (h: any) => h(App),
}).$mount('#app')
