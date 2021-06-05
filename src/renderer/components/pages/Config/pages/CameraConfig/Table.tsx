import React, { FC, useCallback } from 'react'
import { useDispatch } from 'react-redux'
import {
  selectAllCamera,
  feedSlice,
} from '@/renderer/store/modules/feed/reducer'
import {
  ICameraData,
  CameraType,
  ICameraFeed,
} from '@/renderer/store/modules/feed/@types'
import { FaTimes } from 'react-icons/fa'
import { useSelector } from '@/renderer/utils/hooks/typedUseSelector'
import { StyledTable } from './Table.styles'
import { Button } from '@/renderer/components/common/Button'
import { Modal } from '@/renderer/components/common/Modal/Modal'
import { useOpenClose } from '@/renderer/utils/hooks/useOpenClose'
import { useService } from '@xstate/react'
import { rosService, videoUrlSelector } from '@/renderer/state/ros'
import { Select } from '@/renderer/components/common/Select'
import { Input } from '@/renderer/components/common/Input'
import { useEscape } from '@/renderer/utils/hooks/useEscape'

interface TableRowProps {
  feed: ICameraFeed
  updateCamera: (
    id: string
  ) => (field: keyof ICameraData, value: string) => void
}

const TableRow: FC<TableRowProps> = ({ feed, updateCamera }) => {
  const {
    id,
    camera: { name, topic, type },
  } = feed

  const dispatch = useDispatch()
  const removeCamera = () => dispatch(feedSlice.actions.removeFeed(id))
  const updateCameraId = updateCamera(id)
  const [isOpen, open, close] = useOpenClose()

  useEscape(() => {
    if (isOpen) {
      close()
    }
  })

  const [state] = useService(rosService)
  const snapshotSource = videoUrlSelector(
    feed.camera,
    'snapshot'
  )(state.context)

  return (
    <tr>
      <td>
        <Input
          value={name}
          onChange={(e) => updateCameraId('name', e.target.value)}
        />
      </td>
      <td>
        <Input
          value={topic}
          onChange={(e) => updateCameraId('topic', e.target.value)}
        />
      </td>
      <td>
        <Select
          value={type}
          options={Object.keys(CameraType).map((o) => ({
            key: o,
            value: o.toLowerCase(),
          }))}
          onChange={(e) => updateCameraId('type', e.target.value)}
        />
      </td>
      <td>
        <Button onClick={open}>Test</Button>
        <Modal
          isOpen={isOpen}
          onClose={close}
          title={`Preview snapshot for: ${name}`}
        >
          <img src={snapshotSource} alt="video server snapshot" />
        </Modal>
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
    (id: string) =>
      (field: keyof ICameraData, value: string): void => {
        const feed = allCameras.find((f) => f.id === id)

        if (!feed) {
          return
        }

        const newCam: ICameraData = { ...feed.camera }
        switch (field) {
          case 'name':
          case 'topic':
            newCam[field] = value
            break
          case 'type':
            newCam[field] =
              CameraType[value as keyof typeof CameraType] || value
            break
        }

        dispatch(feedSlice.actions.updateCamera({ camera: newCam, id }))
      },
    [allCameras, dispatch]
  )

  return (
    <StyledTable>
      <thead>
        <tr>
          <th>Name</th>
          <th>Topic</th>
          <th>Type</th>
          <th>Snapshot</th>
          <th />
        </tr>
      </thead>
      <tbody>
        {allCameras.map((feed) => (
          <TableRow key={feed.id} feed={feed} updateCamera={updateCamera} />
        ))}
      </tbody>
    </StyledTable>
  )
}
