import { Button } from '@/renderer/components/common/Button'
import { feedSlice } from '@/renderer/store/modules/feed'
import {
  launchFilesSlice,
  selectAllLaunchFiles,
} from '@/renderer/store/modules/launchFiles'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import React, { FC } from 'react'
import { useDispatch, useSelector } from 'react-redux'
import ROSLIB, { Message } from 'roslib'
import { LaunchElement } from './LaunchElement'
import { toast } from 'react-toastify'

export const LaunchConfig: FC = () => {
  const dispatch = useDispatch()

  const onClick = (fileName: string, packageName: string) => {
    dispatch(launchFilesSlice.actions.launchFile(fileName)) //This should be in subscriber callback and called if launch is successful
    launchHanlderTopic.publish({ data: packageName + ' ' + fileName })
  }

  const onClickAll = () => {
    allLaunchFiles.forEach((element) => {
      onClick(element.fileName, element.packageName)
    })
  }

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
  name: 'launchHandler/',
  messageType: 'std_msgs/String',
})

const launchConfirmationTopic = new ROSLIB.Topic({
  ros: rosClient.ros,
  name: 'launchConfirmation/',
  messageType: 'std_msgs/String',
})

const subscriberCallBack = (launched: Message) => {
  if (launched) {
    toast.success('File launched')
  } else {
    toast.error('File not launched (see debug for more info)')
  }
}

launchHanlderTopic.advertise()
launchConfirmationTopic.subscribe((launched: Message) =>
  subscriberCallBack(launched)
)
