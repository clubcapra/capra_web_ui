import RosClient from './roslib-ts-client/RosClient'
import { rosService } from 'state/ros'

const { IP, port } = rosService.state.context
export const rosClient = new RosClient(IP, port)

rosClient.setListeners({
  onConnection: () => {
    rosService.send({ type: 'SUCCESS' })
  },
  onClose: () => {
    rosService.send({ type: 'DISCONNECT' })
  },
  onError: () => {
    rosService.send({ type: 'FAIL' })
  },
})
