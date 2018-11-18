const state = {
  teleop: {
    camera1: {
      type: 'mjpeg',
      topic: '/capra/camera_3d/rgb/image_raw'
    }
  }
}

const mutations = {
  SET_TELEOP_CAMERA1_TOPIC(state, topic) {
    state.teleop.camera1.topic = topic
  }
}

const actions = {
  setTeleopCamera1Topic({ commit }, topic) {
    commit('SET_TELEOP_CAMERA1_TOPIC', topic)
  }
}

export default {
  namespaced: true,
  state,
  mutations,
  actions
}
