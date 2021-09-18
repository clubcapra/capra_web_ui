const mockRequire = (arg: string) => {
  switch (arg) {
    case 'electron':
      return {
        ipcRenderer: {
          on: () => {
            return {}
          },
        },
      }
  }
}
window.require = mockRequire as NodeRequire

import React from 'react'
import ReactDOM from 'react-dom'
import { App } from '../App'
import { act } from 'react-dom/test-utils'

it('renders without crashing ReactDOM', () => {
  const div = document.createElement('div')
  act(() => {
    ReactDOM.render(<App />, div)
  })
  ReactDOM.unmountComponentAtNode(div)
})
