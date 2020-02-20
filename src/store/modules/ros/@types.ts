import { ReactText } from 'react'

export interface RosState {
  IP: string
  port: string
  videoServerPort: string
  connected: boolean
  error: unknown
  tryingToConnect: boolean
  connectingToastId: ReactText
  errorToastId: ReactText
  descriptionServerPort: string
  baseLinkName: string
}
