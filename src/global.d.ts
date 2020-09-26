import { IpcRenderer } from 'electron'

// This is a hack to support png since typescript doesn't know what to do with them
declare module '*.png' {
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const value: any
  export = value
}

// This makes it possible to import the ipcRenderer fomr the renderer otherwise typescript complains a lot
declare global {
  interface Window {
    require: (
      module: 'electron'
    ) => {
      ipcRenderer: IpcRenderer
    }
  }
}
