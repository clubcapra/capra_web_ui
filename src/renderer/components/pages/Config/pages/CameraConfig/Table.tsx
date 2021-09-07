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
import { useEscape } from '@/renderer/hooks/useEscape'
import { useOpenClose } from '@/renderer/hooks/useOpenClose'
import React, { FC, useCallback } from 'react'
import { FaTimes } from 'react-icons/fa'
import { useDispatch } from 'react-redux'

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
  const snapshotSource = useSelector(selectVideoUrl(feed.camera, 'snapshot'))
  const [isOpen, open, close] = useOpenClose()

  useEscape(() => {
    if (isOpen) {
      close()
    }
  })

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
