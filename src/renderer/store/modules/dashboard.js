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
  },
  speed: {
    topic: {
      name: '/capra/speed',
      messageType: 'vel'
    },
    data: 2
  }
}

const mutations = {
  SET_ORIENTATION(state, newOrientation) {
    state.orientation.data = newOrientation
  },
  SET_TEMPERATURE(state, newTemperature) {
    state.temperature.data = newTemperature
  },
  SET_SPEED(state, newSpeed) {
    state.speed.data = newSpeed
  }
}

const actions = {
  updateOrientation({ commit }, orientation) {
    commit('SET_ORIENTATION', orientation)
  },
  updateTemperature({ commit }, temperature) {
    commit('SET_TEMPERATURE', temperature)
  },
  updateSpeed({ commit }, speed) {
    commit('SET_SPEED', speed)
  }
}

export default {
  namespaced: true,
  state,
  mutations,
  actions
}
