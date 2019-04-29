import Vue from 'vue'
import Vuex, { Store, StoreOptions } from 'vuex'

Vue.use(Vuex)

interface RootState {
  isDebug: boolean
}

const store: StoreOptions<RootState> = {
  strict: process.env.NODE_ENV !== 'production',
  state: {
    isDebug: true,
  },
}

export default new Store<RootState>(store)
