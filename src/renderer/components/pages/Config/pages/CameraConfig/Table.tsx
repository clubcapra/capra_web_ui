import { Button } from '@/renderer/components/common/Button'
import { Input } from '@/renderer/components/common/Input'
import { Modal } from '@/renderer/components/common/Modal'
import { Select } from '@/renderer/components/common/Select'
import { styled } from '@/renderer/globalStyles/styled'
import {
  CameraType,
  ICameraData,
  ICameraFeed,
  feedSlice,
  selectAllCamera,
} from '@/renderer/store/modules/feed'
import { selectVideoUrl } from '@/renderer/store/modules/ros'
import { useSelector } from '@/renderer/hooks/typedUseSelector'
import { useOpenClose } from '@/renderer/hooks/useOpenClose'
import React, { FC, useCallback } from 'react'
import { CgTrash } from 'react-icons/cg'
import { useDispatch } from 'react-redux'
import { useKeyPress } from '@/renderer/hooks/useKeyPress'

interface TableRowProps {
  feed: ICameraFeed
  updateCamera: (
    id: string
  ) => (field: keyof ICameraData, value: string | boolean) => void
}

const TableRow: FC<TableRowProps> = ({ feed, updateCamera }) => {
  const {
    id,
    camera: { name, topic, type, flipped, rotated },
  } = feed

  const dispatch = useDispatch()
  const removeCamera = () => dispatch(feedSlice.actions.removeFeed(id))
  const updateCameraId = updateCamera(id)
  const snapshotSource = useSelector(selectVideoUrl(feed.camera, 'snapshot'))
  const [isOpen, open, close] = useOpenClose()

  useKeyPress('Escape', () => {
    if (isOpen) {
      close()
    }
  })

  return (
    <tr>
      <td>
        <Input
          type="text"
          value={name}
          onChange={(e) => updateCameraId('name', e.target.value)}
        />
      </td>
      <td>
        <Input
          type="text"
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
        <Input
          type="checkbox"
          value={flipped}
          onChange={(e) => updateCameraId('flipped', e.target.checked)}
        />
      </td>
      <td>
        <Input
          type="checkbox"
          value={rotated}
          onChange={(e) => updateCameraId('rotated', e.target.checked)}
        />
      </td>
      <td>
        <div onClick={removeCamera}>
          <CgTrash color="red" />
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
      (field: keyof ICameraData, value: string | boolean): void => {
        const feed = allCameras.find((f) => f.id === id)

        if (!feed) {
          return
        }

        const newCam: ICameraData = { ...feed.camera }
        switch (field) {
          case 'name':
          case 'topic':
            newCam[field] = value as string
            break
          case 'type':
            newCam[field] =
              CameraType[value as keyof typeof CameraType] || value
            break
          case 'flipped':
          case 'rotated':
            newCam[field] = value as boolean
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
          <th title="Flips the image horizontally (mirror)">Flipped</th>
          <th title="Rotated the image by 180 degrees">Rotated</th>
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

const StyledTable = styled.table`
  width: 100%;
  border-spacing: 0;
  border-collapse: collapse;

  th,
  td {
    text-align: left;

    &:last-child {
      width: 32px;
    }
  }

  td {
    padding: 8px 8px;

    &:last-child {
      cursor: pointer;
    }

    input {
      width: 100%;
    }

    select {
      width: 100%;
    }
  }

  thead th {
    padding: 4px 8px;
    font-size: inherit;
    border-bottom: 1px solid ${({ theme }) => theme.colors.border};
  }

  tbody {
    border: 1px solid ${({ theme }) => theme.colors.border};
  }
`
