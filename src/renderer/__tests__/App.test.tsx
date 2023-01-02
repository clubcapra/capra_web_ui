import { preload } from '@/main/preload/api';
jest.mock('@/main/preload');

window.preloadApi = {
  ...preload,
  app_info: { appName: '', appVersion: process.env.npm_package_version || '' },
};

import React from 'react';
import ReactDOM from 'react-dom';
import { App } from '../App';
import { act } from 'react-dom/test-utils';

it('renders without crashing ReactDOM', () => {
  const div = document.createElement('div');
  act(() => {
    ReactDOM.render(<App />, div);
  });
  ReactDOM.unmountComponentAtNode(div);
});
