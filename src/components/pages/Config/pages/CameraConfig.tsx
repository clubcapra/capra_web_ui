import React, { ChangeEvent } from 'react'
import { useDispatch } from 'react-redux'
import {
  FeedTypeEnum,
  ICameraData,
  CameraType,
} from 'store/modules/feed/@types'
import { FaTimes } from 'react-icons/fa'
import { useSelector } from 'utils/hooks/typedUseSelector'
import {
  CameraConfigWrapper,
  CameraConfigHeaderWrapper,
  CameraAddButton,
  CameraConfigTable,
  StyledTableInput,
} from 'components/pages/Config/pages/CameraConfig.styles'
import { feedSlice, selectAllCamera } from 'store/modules/feed/reducer'

export const CameraConfig = () => {
  const dispatch = useDispatch()
  const allCameras = useSelector(selectAllCamera)

  const removeCamera = (id: string) =>
    dispatch(feedSlice.actions.removeFeed(id))

  const updateCamera = (id: string, field: keyof ICameraData) => ({
    target: { value },
  }: ChangeEvent<HTMLInputElement>) => {
    const feed = allCameras.find(f => f.id === id)

    if (!feed || feed.type !== FeedTypeEnum.camera) return

    const newCam: ICameraData = { ...feed.camera }
    switch (field) {
      case 'name':
      case 'topic':
        newCam[field] = value
        break
      case 'type':
        newCam[field] = (CameraType as any)[value]
        break
    }
    dispatch(feedSlice.actions.changeCamera({ camera: newCam, id }))
  }
  const handleAddFeed = () =>
    dispatch(
      feedSlice.actions.addCamera({
        name: '',
        topic: '',
        type: CameraType.MJPEG,
      })
    )

  return (
    <CameraConfigWrapper>
      <CameraConfigHeaderWrapper>
        <h1>Cameras</h1>
        <CameraAddButton onClick={handleAddFeed}>Add</CameraAddButton>
      </CameraConfigHeaderWrapper>
      <CameraConfigTable>
        <thead>
          <tr>
            <th>Name</th>
            <th>Topic</th>
            <th>Type</th>
            <th />
          </tr>
        </thead>
        <tbody>
          {allCameras.map(feed => (
            <tr key={feed.id}>
              <td>
                <StyledTableInput
                  value={feed.camera.name}
                  onChange={updateCamera(feed.id, 'name')}
                />
              </td>
              <td>
                <StyledTableInput
                  value={feed.camera.topic}
                  onChange={updateCamera(feed.id, 'topic')}
                />
              </td>
              <td>
                <StyledTableInput
                  value={feed.camera.type}
                  onChange={updateCamera(feed.id, 'type')}
                />
              </td>
              <td>
                <div onClick={() => removeCamera(feed.id)}>
                  <FaTimes />
                </div>
              </td>
            </tr>
          ))}
        </tbody>
      </CameraConfigTable>
    </CameraConfigWrapper>
  )
}
