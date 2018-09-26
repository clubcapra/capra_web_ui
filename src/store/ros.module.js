import {
  CONNECT,
  DISCONNECT,
  UPDATE_ORIENTATION,
  UPDATE_ROBOTIP,
  UPDATE_TEMPERATURE
} from '@/store/actions.type'
import {
  SET_CONNECT,
  SET_ROBOTIP,
  SET_ORIENTATION,
  SET_TEMPERATURE
} from '@/store/mutations.type'

const state = {
  connected: false,
  robotIP: 'localhost',
  orientation: {
    x: 0,
    y: 0,
    z: 0
  },
  temperature: 0
}

const getters = {
  cameras(state) {
    let getCameraURL = topic =>
      `http://${state.robotIP}:8080/stream?topic=${topic}&type=mjpeg`
    let camera3d = '/capra/camera_3d/'
    return {
      front: {
        depth: getCameraURL(camera3d + 'depth/image'),
        rgb: getCameraURL(camera3d + 'rgb/image_raw')
      },
      rear: {
        rgb: getCameraURL('/capra/camera/rgb/image_raw')
      }
    }
  }
}

const actions = {
  [CONNECT]({ commit }) {
    commit(SET_CONNECT, true)
  },
  [DISCONNECT]({ commit }) {
    commit(SET_CONNECT, false)
  },
  [UPDATE_ROBOTIP]({ commit }, robotIP) {
    commit(SET_ROBOTIP, robotIP)
  },
  [UPDATE_ORIENTATION]({ commit }, orientation) {
    commit(SET_ORIENTATION, orientation)
  },
  [UPDATE_TEMPERATURE]({ commit }, temperature) {
    commit(SET_TEMPERATURE, temperature)
  }
}

const mutations = {
  [SET_CONNECT](state, connected) {
    state.connected = connected
  },
  [SET_ROBOTIP](state, IP) {
    state.robotIP = IP
  },
  [SET_ORIENTATION](state, orientation) {
    state.orientation = orientation
  },
  [SET_TEMPERATURE](state, temperature) {
    state.temperature = temperature
  }
}

export default {
  namespaced: true,
  state,
  getters,
  mutations,
  actions
}
