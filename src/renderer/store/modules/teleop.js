const state = {
  camera1: {
    type: 'mjpeg',
    topic: '/capra/camera_3d/rgb/image_raw'
  },
  camera2: {
    type: 'mjpeg',
    topic: '/capra/camera_3d/depth/image_raw'
  }
}

const mutations = {
  SET_TOPIC(state, { cameraName, topic }) {
    state[cameraName].topic = topic
  },
  SET_TYPE(state, cameraName, type) {
    state[cameraName].type = type
  }
}

const actions = {
  setTopic({ commit }, { cameraName, topic }) {
    commit('SET_TOPIC', { cameraName, topic })
  },
  setType({ commit }, cameraName, type) {
    commit('SET_TYPE', cameraName, type)
  }
}

export default {
  namespaced: true,
  state,
  mutations,
  actions
}
