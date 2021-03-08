import RosClient from './roslib-ts-client/RosClient'
import { rosService } from '@/renderer/state/ros'

const { IP, port } = rosService.state.context
export const rosClient = new RosClient(IP, port)

rosClient.setListeners({
  onConnection: () => {
    rosService.send('SUCCESS')
  },
  onClose: () => {
    rosService.send('DISCONNECT')
  },
  onError: () => {
    rosService.send('FAIL')
  },
})
