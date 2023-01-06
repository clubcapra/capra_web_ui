import { CameraType, ICameraData } from '@/renderer/store/modules/feed';
import { GlobalState } from '@/renderer/store/store';
import { createSlice, PayloadAction } from '@reduxjs/toolkit';

export interface RosState {
  namespace: string;
  IP: string;
  port: string;
  videoServerPort: string;
  descriptionServerPort: string;
  baseLinkName: string;
}

export const initialState: RosState = {
  namespace: 'capra_ui/',
  IP: 'jetson',
  port: '9090',
  videoServerPort: '8080',
  descriptionServerPort: '88',
  baseLinkName: 'markhor_link_base',
};

export const rosSlice = createSlice({
  name: 'ros',
  initialState,
  reducers: {
    updateNamespace: (state, { payload }: PayloadAction<string>) => {
      state.namespace = payload;
    },
    updateIP: (state, { payload }: PayloadAction<string>) => {
      state.IP = payload;
    },
    updatePort: (state, { payload }: PayloadAction<string>) => {
      state.port = payload;
    },
    updateVideoServerPort: (state, { payload }: PayloadAction<string>) => {
      state.videoServerPort = payload;
    },
    updateDescriptionServerPort: (
      state,
      { payload }: PayloadAction<string>
    ) => {
      state.descriptionServerPort = payload;
    },
    updateBaseLinkName: (state, { payload }: PayloadAction<string>) => {
      state.baseLinkName = payload;
    },
  },
});

export const selectIP = (state: GlobalState): string => state.ros.IP;
export const selectPort = (state: GlobalState): string => state.ros.port;
export const selectVideoServerPort = (state: GlobalState): string =>
  state.ros.videoServerPort;
export const selectDescriptionServerPort = (state: GlobalState): string =>
  state.ros.descriptionServerPort;
export const selectBaseLinkName = (state: GlobalState): string =>
  state.ros.baseLinkName;

export const selectNamespace = (state: GlobalState): string =>
  state.ros.namespace;
export const selectFullAddress = (state: GlobalState): string =>
  `${state.ros.IP}:${state.ros.port}/`;

export const selectVideoUrl =
  (camera: ICameraData, param: 'stream' | 'snapshot' = 'stream') =>
  (state: GlobalState): string => {
    let cameraType = camera.type;
    // QR Code feeds are streamed as mjpeg
    if (camera.type === CameraType.QR_CODE) {
      cameraType = CameraType.MJPEG;
    }

    return `http://${state.ros.IP}:${state.ros.videoServerPort}/${param}?topic=${camera.topic}&type=${cameraType}`;
  };
