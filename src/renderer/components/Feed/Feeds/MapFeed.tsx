import React, { FC, useEffect, useLayoutEffect, useRef, useState } from 'react'
import { styled } from '@/renderer/globalStyles/styled'
import * as ROS2D from 'ros2d'
import { useRefSize } from '@/renderer/utils/hooks/useRefSize'
import _ from 'lodash'
import { rosClient } from '@/renderer/utils/ros/rosClient'

const StyledViewer = styled.div`
  height: 100%;
  width: 100%;
  overflow: hidden;
`

function use2dViewerRef(): [
  ROS2D.Viewer | undefined,
  string,
  React.RefObject<HTMLDivElement>
] {
  const id = _.uniqueId('urdf-viewer-')
  const ref = useRef<HTMLDivElement>(null)
  const [width, height] = useRefSize(ref)
  const [viewer, setViewer] = useState<ROS2D.Viewer>()

  useEffect(() => {
    const localViewer = new ROS2D.Viewer({
      divID: id,
      width: width - 1,
      height: height - 1,
    })
    setViewer(localViewer)
  }, [viewer, id])

  useLayoutEffect(() => {
    viewer?.scaleToDimensions(width - 1, height - 1)
  }, [width, height, viewer])

  return [viewer, id, ref]
}

export const MapFeed: FC = () => {
  const [viewer, id, ref] = use2dViewerRef()

  useEffect(() => {
    if (!viewer) {
      return
    }
    const ros = rosClient.ros
    const gridClient = new ROS2D.OccupancyGridClient({
      ros: ros,
      rootObject: viewer.scene,
    })
  }, [viewer])
  return <StyledViewer />
}
