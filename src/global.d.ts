import { IpcRenderer } from 'electron'

// This makes it possible to import the ipcRenderer from the renderer otherwise typescript complains a lot
declare global {
  interface Window {
    require: (
      module: 'electron'
    ) => {
      ipcRenderer: IpcRenderer
    }
  }
}
