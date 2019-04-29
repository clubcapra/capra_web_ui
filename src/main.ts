import Vue from 'vue'
import vueBulmaComponents from 'vue-bulma-components'
import { library } from '@fortawesome/fontawesome-svg-core'
import { faCircle } from '@fortawesome/free-solid-svg-icons'
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'

import App from './App.vue'
import router from './router'
import store from './store'
import '@/utils/gamepad/GamepadManager'

library.add(faCircle)

Vue.component('font-awesome-icon', FontAwesomeIcon)

Vue.config.productionTip = false

Vue.use(vueBulmaComponents)

new Vue({
  router,
  store,
  render: h => h(App),
}).$mount('#app')
