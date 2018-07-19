import RosClient from '@/lib/RosClient'
import { mapState, mapActions } from 'vuex'
import { rosActions } from '@/store/modules/ros'

export default {
  name: 'Main',
  computed: mapState({
    connected: state => state.ros.connected,
    error: state => state.ros.error
  }),
  methods: {
    ...mapActions('ros', {
      handleConnect: rosActions.CONNECT,
      handleDisconnect: rosActions.DISCONNECT,
      handleError: rosActions.ERROR
    })
  },
  mounted() {
    let rosclient = new RosClient()
    rosclient.setListeners({
      connect: this.handleConnect,
      disconnect: this.handleDisconnect,
      error: this.handleError
    })
    rosclient.connect()
  }
}
