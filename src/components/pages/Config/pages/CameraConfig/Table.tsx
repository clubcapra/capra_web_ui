import React, { FC, useCallback, ChangeEvent } from 'react'
import { useDispatch } from 'react-redux'
import { selectAllCamera, feedSlice } from 'store/modules/feed/reducer'
import { ICameraData, CameraType, ICameraFeed } from 'store/modules/feed/@types'
import { FaTimes } from 'react-icons/fa'
import { useSelector } from 'utils/hooks/typedUseSelector'
import { StyledCameraConfigTable, StyledTableInput } from './Table.styles'

interface TableRowProps {
  feed: ICameraFeed
  updateCamera: (
    id: string
  ) => (field: keyof ICameraData) => (e: ChangeEvent<HTMLInputElement>) => void
}

const TableRow: FC<TableRowProps> = ({ feed, updateCamera }) => {
  const {
    id,
    camera: { name, topic, type },
  } = feed

  const dispatch = useDispatch()

  const removeCamera = useCallback(
    () => dispatch(feedSlice.actions.removeFeed(id)),
    [dispatch, id]
  )

  const updateCameraId = useCallback(updateCamera(id), [id])

  return (
    <tr>
      <td>
        <StyledTableInput value={name} onChange={updateCameraId('name')} />
      </td>
      <td>
        <StyledTableInput value={topic} onChange={updateCameraId('topic')} />
      </td>
      <td>
        <StyledTableInput value={type} onChange={updateCameraId('type')} />
      </td>
      <td>
        <div onClick={removeCamera}>
          <FaTimes />
        </div>
      </td>
    </tr>
  )
}

export const Table: FC = () => {
  const dispatch = useDispatch()
  const allCameras = useSelector(selectAllCamera)

  const updateCamera = useCallback(
    (id: string) => (field: keyof ICameraData) => (
      e: ChangeEvent<HTMLInputElement>
    ): void => {
      const { value } = e.target
      const feed = allCameras.find(f => f.id === id)

      if (!feed) return

      const newCam: ICameraData = { ...feed.camera }
      switch (field) {
        case 'name':
        case 'topic':
          newCam[field] = value
          break
        case 'type':
          // eslint-disable-next-line @typescript-eslint/no-explicit-any
          newCam[field] = (CameraType as any)[value]
          break
      }

      dispatch(feedSlice.actions.updateCamera({ camera: newCam, id }))
    },
    [allCameras, dispatch]
  )

  return (
    <StyledCameraConfigTable>
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
          <TableRow key={feed.id} feed={feed} updateCamera={updateCamera} />
        ))}
      </tbody>
    </StyledCameraConfigTable>
  )
}
