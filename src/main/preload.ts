import { contextBridge } from 'electron';

export const APP_INFO_QUERY = 'app_info_query';
export type APP_INFO_TYPE = { appName: string; appVersion: string };

export const LOG_MSG = 'log_mmsg';
export type LOG_MSG_TYPE = {
  level: 'error' | 'warn' | 'info' | 'debug';
  message: string;
};

export const AUDIO_START = 'audio_start';
export const AUDIO_STOP = 'audio_stop';
export const AUDIO_MSG = 'audio_msg';
export type AUDIO_MSG_TYPE = {
  error: string | undefined;
  stderr: string;
  stdout: string;
};

if (contextBridge) {
  // This is done this way because otherwise tests would try to import preload and it wouldn't work
  // It's possible the issue is mostly in the import order.
  // I think it happens because the exported constant are imported in multiple context where they shouldn't be
  // eslint-disable-next-line @typescript-eslint/no-var-requires, @typescript-eslint/no-unsafe-assignment
  const { preload } = require('@/main/preload/api');
  contextBridge.exposeInMainWorld('preloadApi', preload);
}
