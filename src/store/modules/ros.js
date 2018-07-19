export const rosActions = {
  CONNECT: 'CONNECT',
  DISCONNECT: 'DISCONNECT',
  ERROR: 'ERROR'
}

const ros = {
  namespaced: true,
  state: {
    connected: false,
    error: null
  },
  mutations: {
    connect(state) {
      state.connected = true
    },
    disconnect(state) {
      state.connected = false
    },
    error(state, payload) {
      state.error = payload
    }
  },
  actions: {
    [rosActions.CONNECT]({ commit }) {
      commit('connect')
    },
    [rosActions.DISCONNECT]({ commit }) {
      commit('disconnect')
    },
    [rosActions.ERROR]({ commit }, payload) {
      commit('error', payload)
    }
  }
}

export default ros
