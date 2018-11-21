const state = {
  camera3d_rgb: {
    type: 'mjpeg',
    topic: '/capra/camera_3d/rgb/image_raw'
  },
  camera3d_depth: {
    type: 'mjpeg',
    topic: '/capra/camera_3d/depth/image_raw'
  }
}

const SET_TOPIC = 'SET_TOPIC'

const mutations = {
  [SET_TOPIC](state, { cameraName, topic }) {
    state[cameraName].topic = topic
  },
  SET_TYPE(state, { cameraName, type }) {
    state[cameraName].type = type
  },
  ADD_CAMERA(
    state,
    {
      cameraName,
      options: { type = 'mjpeg', topic = '' }
    }
  ) {
    state = { ...state, [cameraName]: { type, topic } }
  }
}

const actions = {
  setTopic({ commit }, { cameraName, topic }) {
    commit(SET_TOPIC, { cameraName, topic })
  },
  setType({ commit }, { cameraName, type }) {
    commit('SET_TYPE', { cameraName, type })
  },
  addCamera({ commit }, { cameraName, options }) {
    commit('ADD_CAMERA', { cameraName, options })
  }
}

export default {
  namespaced: true,
  state,
  mutations,
  actions
}
