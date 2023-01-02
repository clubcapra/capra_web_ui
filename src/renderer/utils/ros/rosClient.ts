import RosClient from './roslib-ts-client/RosClient';
import { rosService } from '@/renderer/state/ros';
import { store } from '@/renderer/store/store';
import { selectIP, selectPort } from '@/renderer/store/modules/ros';

const IP = selectIP(store.getState());
const port = selectPort(store.getState());
export const rosClient = new RosClient(IP, port);

rosClient.setListeners({
  onConnection: () => {
    rosService.send('SUCCESS');
  },
  onClose: () => {
    rosService.send('DISCONNECT');
  },
  onError: () => {
    rosService.send('FAIL');
  },
});
