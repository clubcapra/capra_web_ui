export const rosActions = {
  CONNECT: 'CONNECT',
  DISCONNECT: 'DISCONNECT',
  ERROR: 'ERROR',
  UPDATE_ORIENTATION: 'UPDATE_ORIENTATION',
  UPDATE_TEMPERATURE: 'UPDATE_TEMPERATURE',
  UPDATE_IP: 'UPDATE_IP'
}

export default {
  namespaced: true,
  state: {
    connected: false,
    error: null,
    robotIP: '192.168.1.120',
    orientation: { x: 0, y: 0, z: 0 },
    temperature: 0,
    camera: {
      front: {
        depth: '',
        thermal: '',
        rgb: ''
      },
      rear: {
        depth: '',
        rgb: ''
      }
    }
  },
  mutations: {
    connect(state) {
      state.connected = true

      let getCameraURL = topic =>
        `http://${state.robotIP}:8080/stream?topic=${topic}`

      let cameras = {
        front: {
          depth: getCameraURL('/capra/camera_3d/depth/image'),
          thermal: '',
          rgb: getCameraURL('/capra/camera_3d/rgb/image')
        },
        rear: {
          depth: '',
          rgb: ''
        }
      }

      state.camera = cameras
    },
    disconnect(state) {
      state.connected = false
    },
    error(state, payload) {
      state.error = payload
    },
    updateOrientation(state, orientation) {
      state.orientation = orientation
    },
    updateTemperature(state, temp) {
      state.temperature = temp
    },
    updateIP(state, robotIP) {
      state.robotIP = robotIP
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
    [rosActions.UPDATE_ORIENTATION]({ commit }, payload) {
      commit('updateOrientation', payload)
    },
    [rosActions.UPDATE_TEMPERATURE]({ commit }, payload) {
      commit('updateTemperature', payload)
    },
    [rosActions.UPDATE_IP]({ commit }, payload) {
      commit('updateIP', payload)
    }
  }
}
