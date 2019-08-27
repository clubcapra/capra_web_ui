import { ReactText } from 'react'

export interface RosState {
  IP: string
  port: string
  connected: boolean
  error: unknown
  tryingToConnect: boolean
  connectingToastId: ReactText
  errorToastId: ReactText
}
