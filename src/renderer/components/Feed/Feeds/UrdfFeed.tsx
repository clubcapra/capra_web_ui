import { NoFeed } from '@/renderer/components/Feed/Feeds/NoFeed'
import { styled } from '@/renderer/globalStyles/styled'
import { rosService } from '@/renderer/state/ros'
import { IUrdfFeed } from '@/renderer/store/modules/feed'
import {
  selectBaseLinkName,
  selectDescriptionServerPort,
  selectIP,
} from '@/renderer/store/modules/ros'
import { useRefSize } from '@/renderer/utils/hooks/useRefSize'
import { rosClient } from '@/renderer/utils/ros/rosClient'
import { useActor } from '@xstate/react'
import _ from 'lodash'
import React, { FC, useEffect, useLayoutEffect, useRef, useState } from 'react'
import { useSelector } from 'react-redux'
import * as ROS3D from 'ros3d'
import ROSLIB from 'roslib'

interface Props {
  feed: IUrdfFeed
}

const Grid = styled.div`
  display: grid;
  height: 100%;
  width: 100%;
  align-items: center;
  justify-items: center;
  background-color: black;
`

const StyledViewer = styled.div`
  height: 100%;
  width: 100%;
  overflow: hidden;
`

function useUrdfViewerRef(): [
  ROS3D.Viewer | undefined,
  string,
  React.RefObject<HTMLDivElement>
] {
  const id = _.uniqueId('urdf-viewer-')
  const ref = useRef<HTMLDivElement>(null)
  const [width, height] = useRefSize(ref)
  const [viewer, setViewer] = useState<ROS3D.Viewer>()
  const [isInit, setIsInit] = useState(false)

  useEffect(() => {
    if (isInit) {
      return
    }

    const localViewer = new ROS3D.Viewer({
      divID: id,
      width: width - 1,
      height: height - 1,
      antialias: true,
    })
    setViewer(localViewer)
    setIsInit(true)
  }, [height, id, isInit, width])

  useLayoutEffect(() => {
    viewer?.resize(width - 1, height - 1)
  }, [width, height, viewer])

  return [viewer, id, ref]
}

const View: FC<Props> = () => {
  const [viewer, id, ref] = useUrdfViewerRef()

  const descriptionServerPort = useSelector(selectDescriptionServerPort)
  const baseLinkName = useSelector(selectBaseLinkName)

  const IP = useSelector(selectIP)

  useEffect(() => {
    if (!viewer) {
      return
    }

    const ros = rosClient.ros

    viewer.addObject(new ROS3D.Grid())

    const tfClient = new ROSLIB.TFClient({
      ros: ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: baseLinkName,
    })

    new ROS3D.UrdfClient({
      ros: ros,
      tfClient: tfClient,
      path: `http://${IP}:${descriptionServerPort}`,
      rootObject: viewer.scene,
    })
  }, [IP, baseLinkName, descriptionServerPort, viewer])

  return <StyledViewer id={id} ref={ref} />
}

export const UrdfFeed: FC<Props> = ({ feed }) => {
  const [state] = useActor(rosService)
  return (
    <Grid>
      {state.matches('connected') ? (
        <View feed={feed} />
      ) : (
        <NoFeed text="not connected" />
      )}
    </Grid>
  )
}
