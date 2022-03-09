import { Button } from '@/renderer/components/common/Button'
import {
  launchFilesSlice,
  selectAllLaunchFiles,
} from '@/renderer/store/modules/launchFiles'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import React, { FC, useEffect } from 'react'
import { useDispatch, useSelector } from 'react-redux'
import { LaunchElement } from './LaunchElement'
import { toast } from 'react-toastify'
import { log } from '@/renderer/logger'

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

  const onClick = (fileName: string, packageName: string) => {
    rosClient
      .callService(
        {
          name: '/launchHandler/launchFile',
        },
        { package: packageName, fileName }
      )
      .then((res: unknown) => {
        const messageData = res as LaunchMsg
        if (messageData.isLaunched) {
          dispatch(launchFilesSlice.actions.launchFile(messageData.fileName))
          toast.success(messageData.message)
        } else {
          dispatch(launchFilesSlice.actions.killFile(messageData.fileName))
          toast.error(messageData.message)
        }
      })
      .catch(log.error)
  }

  const onClickAll = () => {
    allLaunchFiles.forEach((element) => {
      onClick(element.fileName, element.packageName)
    })
  }

  const allLaunchFiles = useSelector(selectAllLaunchFiles)

  useEffect(() => {
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
      .catch(log.error)
  }, [allLaunchFiles, dispatch])

  return (
    <>
      <Button onClick={onClickAll}>Launch All</Button>
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
    </>
  )
}
