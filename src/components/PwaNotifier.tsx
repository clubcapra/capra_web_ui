import React, { FC, useEffect } from 'react'
//@ts-ignore
import { useToasts } from 'react-toast-notifications'
import { useSelector } from 'react-redux'
import { GlobalState } from 'store/rootReducer'

export const NotifyOfflineMode: FC = () => {
  const { addToast } = useToasts()

  const offlineMode = useSelector((state: GlobalState) => state.pwa.offlineMode)

  useEffect(() => {
    if (offlineMode) {
      console.log('offline mode is enabled')
      addToast(
        'No internet connection found. App is running in offline mode.',
        {
          appearance: 'info',
          autoDismiss: true,
        }
      )
    }
  }, [addToast, offlineMode])

  return <></>
}

export const PwaNotifier: FC = () => {
  const { addToast } = useToasts()

  const contentLoaded = useSelector(
    (state: GlobalState) => state.pwa.contentLoaded
  )
  const newContentLoaded = useSelector(
    (state: GlobalState) => state.pwa.newContentLoaded
  )
  const offlineMode = useSelector((state: GlobalState) => state.pwa.offlineMode)

  useEffect(() => {
    if (contentLoaded) {
      addToast('Content is cached for offline use.', {
        appearance: 'info',
        autoDismiss: true,
      })
    }
  }, [addToast, contentLoaded])

  useEffect(() => {
    if (newContentLoaded) {
      addToast(
        'New content is available and will be used when all tabs for this page are closed.',
        {
          appearance: 'info',
          autoDismiss: true,
        }
      )
    }
  }, [addToast, newContentLoaded])

  useEffect(() => {
    if (offlineMode) {
      addToast(
        'No internet connection found. App is running in offline mode.',
        {
          appearance: 'info',
          autoDismiss: true,
        }
      )
    }
  }, [addToast, offlineMode])

  return null
}
