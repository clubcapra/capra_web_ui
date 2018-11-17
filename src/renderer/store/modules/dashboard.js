const state = {
  orientation: {
    topic: {
      name: '/capra/imu',
      messageType: 'sensor_msgs/Imu'
    },
    data: {
      x: 0,
      y: 0,
      z: 0
    }
  },
  temperature: {
    topic: {
      name: '/capra/imu',
      messageType: 'sensor_msgs/Imu'
    },
    data: 0
  }
}

const mutations = {
  SET_ORIENTATION(state, newOrientation) {
    state.orientation.data = newOrientation
  },
  SET_TEMPERATURE(state, newTemperature) {
    state.temperature = newTemperature
  }
}

const actions = {
  updateOrientation({ commit }, orientation) {
    commit('SET_ORIENTATION', orientation)
  },
  updateTemperature({ commit }, temperature) {
    commit('SET_TEMPERATURE', temperature)
  }
}

export default {
  namespaced: true,
  state,
  mutations,
  actions
}
