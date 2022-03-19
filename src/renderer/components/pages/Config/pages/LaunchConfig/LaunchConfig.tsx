import { Button } from '@/renderer/components/common/Button'
import {
  launchFilesSlice,
  selectAllLaunchFiles,
} from '@/renderer/store/modules/launchFiles'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import React, { FC, useCallback, useEffect, useState } from 'react'
import { useDispatch, useSelector } from 'react-redux'
import { LaunchElement } from './LaunchElement'
import { toast } from 'react-toastify'
import { log } from '@/renderer/logger'
import { FaSync } from 'react-icons/fa'
import { LoadingOverlay } from '@/renderer/components/common/LoadingOverlay'
import { useActor } from '@xstate/react'
import { rosService } from '@/renderer/state/ros'

interface LaunchMsg {
  fileName: string
  message: string
  isLaunched: boolean
}

interface LaunchedFiles {
  packages: string[]
  fileNames: string[]
}

export const LaunchConfig: FC = () => {
  const dispatch = useDispatch()
  const allLaunchFiles = useSelector(selectAllLaunchFiles)
  const [isLoading, setIsLoading] = useState(false)
  const [connectionState] = useActor(rosService)

  const onClick = useCallback(
    (fileName: string, packageName: string) => {
      setIsLoading(true)
      rosClient
        .callService(
          {
            name: '/launchHandler/launchFile',
          },
          { package: packageName, fileName }
        )
        .then((res: unknown) => {
          const messageData = res as LaunchMsg
          setIsLoading(false)
          if (messageData.isLaunched) {
            dispatch(launchFilesSlice.actions.launchFile(messageData.fileName))
            toast.success(messageData.message)
          } else {
            dispatch(launchFilesSlice.actions.killFile(messageData.fileName))
            toast.error(messageData.message)
          }
        })
        .catch((e: string) => {
          setIsLoading(false)
          toast.error(e)
        })
    },
    [dispatch]
  )

  const onClickLaunchAll = () => {
    allLaunchFiles.forEach((element) => {
      if (!element.isLaunched) {
        onClick(element.fileName, element.packageName)
      }
    })
  }

  const onClickKillAll = useCallback(() => {
    allLaunchFiles.forEach((element) => {
      if (element.isLaunched) {
        onClick(element.fileName, element.packageName)
      }
    })
  }, [allLaunchFiles, onClick])

  const onClickRefresh = () => {
    refreshLaunchedFiles()
  }

  const refreshLaunchedFiles = useCallback(() => {
    rosClient
      .callService({
        name: '/launchHandler/getAllLaunchedFiles',
      })
      .then((res: unknown) => {
        const launchedFiles = res as LaunchedFiles
        for (const element of allLaunchFiles) {
          if (
            !element.isLaunched &&
            launchedFiles.fileNames.includes(element.fileName)
          ) {
            dispatch(launchFilesSlice.actions.launchFile(element.fileName))
          } else if (
            element.isLaunched &&
            !launchedFiles.fileNames.includes(element.fileName)
          ) {
            dispatch(launchFilesSlice.actions.killFile(element.fileName))
          }
        }
      })
      .catch((e: string) => {
        toast.error(e)
      })
  }, [allLaunchFiles, dispatch])

  useEffect(() => {
    if (connectionState.matches('connected')) {
      refreshLaunchedFiles()
    } else {
      allLaunchFiles.forEach((element) => {
        if (element.isLaunched) {
          dispatch(launchFilesSlice.actions.killFile(element.fileName))
        }
      })
    }
  }, [
    allLaunchFiles,
    connectionState,
    dispatch,
    onClickKillAll,
    refreshLaunchedFiles,
  ])

  const stylingObject = {
    div: {
      display: 'flex',
      marginTop: 5,
    },
  }

  return (
    <>
      {connectionState.matches('connected') ? (
        <>
          <div style={stylingObject.div}>
            <Button onClick={onClickLaunchAll}>Launch All</Button>
            <Button onClick={onClickKillAll}>Kill All</Button>
            <Button onClick={onClickRefresh}>
              <FaSync />
            </Button>
          </div>
          <div>
            {allLaunchFiles.map((element) => (
              <LaunchElement
                key={element.name}
                name={element.name}
                launchFile={element.fileName}
                packageName={element.packageName}
                onClick={onClick}
                isLaunched={element.isLaunched}
              />
            ))}
          </div>
          {isLoading ? <LoadingOverlay>File is launching</LoadingOverlay> : ''}
        </>
      ) : (
        <div>No connection to ROS</div>
      )}
    </>
  )
}
