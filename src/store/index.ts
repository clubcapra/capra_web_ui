import Vue from 'vue'
import Vuex, { Store } from 'vuex'

// import modules from './modules'

Vue.use(Vuex)

const store = new Store({
  strict: process.env.NODE_ENV !== 'production',
  // modules,
})

export default store
