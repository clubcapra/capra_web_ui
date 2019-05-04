import Vue from 'vue'
import vueBulmaComponents from 'vue-bulma-components'
import './fontAwesome'

import App from './App.vue'
import router from './router'
import store from './store'
import '@/utils/gamepad/GamepadManager'

Vue.config.productionTip = false

Vue.use(vueBulmaComponents)

new Vue({
  router,
  store,
  render: h => h(App),
}).$mount('#app')
