import { log } from '@/renderer/logger';
import React from 'react';
import ReactDOM from 'react-dom';
import { App } from './App';

if (!window.preloadApi.app_info) {
  // eslint-disable-next-line no-console
  console.error('PreloadApi failed to initialize properly', window.preloadApi);
} else {
  ReactDOM.render(<App />, document.getElementById('root'));

  log.info(
    'App finished loading... appVersion: ',
    window.preloadApi.app_info.appVersion
  );
}
