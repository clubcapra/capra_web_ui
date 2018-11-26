import Vue from 'vue'
import axios from 'axios'
import vueBulmaComponents, {
  bulmaComponentGenerator
} from 'vue-bulma-components'

// import 'bulma/css/bulma.css'
// import 'bulmaswatch/slate/bulmaswatch.scss'

import App from './App'
import router from './router'
import store from './store'
import './requireBaseComponents'

if (!process.env.IS_WEB) Vue.use(require('vue-electron'))

Vue.http = Vue.prototype.$http = axios
Vue.config.productionTip = false

Vue.use(vueBulmaComponents)
Vue.component(bulmaComponentGenerator('card-header-title'))

/* eslint-disable no-new */
new Vue({
  router,
  store,
  render: h => h(App)
}).$mount('#app')
