import { Button } from '@/renderer/components/common/Button'
import { feedSlice } from '@/renderer/store/modules/feed'
import {
  launchFilesSlice,
  selectAllLaunchFiles,
} from '@/renderer/store/modules/launchFiles'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import React, { FC, useCallback, useMemo } from 'react'
import { useDispatch, useSelector } from 'react-redux'
import ROSLIB, { Message } from 'roslib'
import { LaunchElement } from './LaunchElement'
import { toast } from 'react-toastify'
import { useRosSubscribe } from '@/renderer/hooks/useRosSubscribe'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'

interface LaunchMsg {
  fileName: string
  message: string
  isLaunched: boolean
}

export const LaunchConfig: FC = () => {
  const dispatch = useDispatch()

  const onClick = (fileName: string, packageName: string) => {
    launchHanlderTopic.publish({ data: packageName + ' ' + fileName })
  }

  const onClickAll = () => {
    allLaunchFiles.forEach((element) => {
      onClick(element.fileName, element.packageName)
    })
  }

  const confirmationTopic: TopicOptions = useMemo(
    () => ({
      name: '/launchConfirmation',
      messageType: 'std_msgs/String',
    }),
    []
  )

  useRosSubscribe(
    confirmationTopic,
    useCallback((message) => {
      const messageData = JSON.parse(message.data as string) as LaunchMsg
      console.log(messageData)
      if (messageData.isLaunched) {
        dispatch(launchFilesSlice.actions.launchFile(messageData.fileName))
        toast.success(messageData.message)
      } else {
        dispatch(launchFilesSlice.actions.killFile(messageData.fileName))
        toast.error(messageData.message)
      }
    }, [])
  )

  const allLaunchFiles = useSelector(selectAllLaunchFiles)

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

const launchHanlderTopic = new ROSLIB.Topic({
  ros: rosClient.ros,
  name: '/launchHandler',
  messageType: 'std_msgs/String',
})

/*const subscriberCallBack = (msg: Message) => {
  console.log(msg)
  if ((msg as LaunchMsg).isLaunched) {
    toast.success((msg as LaunchMsg).message)
  } else {
    toast.error((msg as LaunchMsg).message)
  }
}*/

launchHanlderTopic.advertise()
/*launchConfirmationTopic.subscribe((launched: Message) =>
  subscriberCallBack(launched)
)*/
