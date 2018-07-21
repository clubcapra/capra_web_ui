export const rosActions = {
  CONNECT: 'CONNECT',
  DISCONNECT: 'DISCONNECT',
  ERROR: 'ERROR',
  UPDATE_TWIST: 'UPDATE_TWIST',
  UPDATE_POSE: 'UPDATE_POSE'
}

export default {
  namespaced: true,
  state: {
    connected: false,
    error: null,
    twist: {},
    pose: {}
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
    },
    updateTwist(state, payload) {
      state.twist = payload
    },
    updatePose(state, payload) {
      state.pose = payload
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
    },
    [rosActions.UPDATE_TWIST]({ commit }, payload) {
      commit('updateTwist', payload)
    },
    [rosActions.UPDATE_POSE]({ commit }, payload) {
      commit('updatePose', payload)
    }
  }
}
