import Vue from 'vue'
import Vuex from 'vuex'

import ros from './ros.module'

Vue.use(Vuex)

const debug = process.env.NODE_ENV !== 'production'

export default new Vuex.Store({
  modules: {
    ros
  },
  strict: debug
})
