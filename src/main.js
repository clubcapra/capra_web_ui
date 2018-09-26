import Vue from 'vue'
import App from './App.vue'
import router from './router'
import store from './store/store.js'
import 'bulma/css/bulma.css'
import '@/assets/themes/slate.css'

Vue.config.productionTip = false

// window.createjs = createjs

new Vue({
  router,
  store,
  render: h => h(App)
}).$mount('#app')
