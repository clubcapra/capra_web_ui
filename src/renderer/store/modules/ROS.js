const state = {
  connected: false,
  robotIP: 'localhost:9090',
  orientation: {
    x: 0,
    y: 0,
    z: 0
  },
  temperature: 0
}

const mutations = {
  CONNECT(state) {
    state.connected = true
  },
  DISCONNECT(state) {
    state.connected = false
  },
  SET_ROBOTIP(state, IP) {
    state.robotIP = IP
  }
}

const actions = {
  connect({ commit }) {
    commit('CONNECT')
  },
  disconnect({ commit }) {
    commit('DISCONNECT')
  },
  setRobotIP({ commit }, robotIP) {
    commit('SET_ROBOTIP', robotIP)
  }
}

export default {
  namespaced: true,
  state,
  mutations,
  actions
}
