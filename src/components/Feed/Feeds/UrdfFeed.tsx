import React, { FC, useRef, useEffect, useLayoutEffect, useState } from 'react'
import { useSelector } from 'utils/hooks/typedUseSelector'
import { styled } from 'globalStyles/styled'
import { NoFeed } from 'components/Feed/Feeds/NoFeed'
import { IUrdfFeed } from 'store/modules/feed/@types'
import { rosClient } from 'utils/ros/rosClient'
import * as ROS3D from 'ros3d'
import { TFClient } from 'roslib'
import { useRefSize } from 'utils/hooks/useRefSize'
import _ from 'lodash'

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
    if (isInit) return

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

const View: FC<Props> = ({ feed }) => {
  const [viewer, id, ref] = useUrdfViewerRef()
  const IP = useSelector(state => state.ros.IP)
  const port = useSelector(state => state.ros.descriptionServerPort)
  const baseLinkName = useSelector(state => state.ros.descriptionServerPort)

  useEffect(() => {
    if (!viewer) return

    const ros = rosClient.ros

    viewer.addObject(new ROS3D.Grid())

    const tfClient = new TFClient({
      ros: ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: baseLinkName,
    })

    new ROS3D.UrdfClient({
      ros: ros,
      tfClient: tfClient,
      path: `http://${IP}:${port}`,
      rootObject: viewer.scene,
    })
  }, [IP, baseLinkName, port, viewer])

  return <StyledViewer id={id} ref={ref} />
}

export const UrdfFeed: FC<Props> = ({ feed }) => {
  const connected = useSelector(state => state.ros.connected)
  return (
    <Grid>{connected ? <View feed={feed} /> : <NoFeed text="no urdf" />}</Grid>
  )
}
