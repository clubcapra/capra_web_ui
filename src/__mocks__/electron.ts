export const ipcRenderer = {
  on: jest.fn(),
  send: jest.fn(),
  sendSync: jest.fn(),
}

export const contextBridge = {
  exposeInMainWorld: jest.fn(),
}
