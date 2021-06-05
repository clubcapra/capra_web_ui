import React, { FC, useCallback } from 'react'
import { useDispatch } from 'react-redux'
import {
  feedSlice,
  selectAllGraph,
} from '@/renderer/store/modules/feed/reducer'
import { IGraphData, IGraphFeed } from '@/renderer/store/modules/feed/@types'
import { FaTimes } from 'react-icons/fa'
import { useSelector } from '@/renderer/utils/hooks/typedUseSelector'
import { StyledTable } from './Table.styles'
import { Input } from '@/renderer/components/common/Input'
import { TopicOptions } from '@/renderer/utils/ros/roslib-ts-client/@types'

interface TableRowProps {
  feed: IGraphFeed
  updateTopic: (
    id: string
  ) => (field: keyof TopicOptions, value: string) => void
  updateName: (id: string) => (value: string) => void
}

const TableRow: FC<TableRowProps> = ({ feed, updateTopic, updateName }) => {
  const {
    id,
    graph: {
      name,
      topic: { name: topicName, messageType },
    },
  } = feed

  const dispatch = useDispatch()
  const removeGraph = () => dispatch(feedSlice.actions.removeFeed(id))
  const updateTopicId = updateTopic(id)
  const updateNameId = updateName(id)

  return (
    <tr>
      <td>
        <Input value={name} onChange={(e) => updateNameId(e.target.value)} />
      </td>
      <td>
        <Input
          value={topicName}
          onChange={(e) => updateTopicId('name', e.target.value)}
        />
      </td>
      <td>
        <Input
          value={messageType}
          onChange={(e) => updateTopicId('messageType', e.target.value)}
        />
      </td>
      <td>
        <div onClick={removeGraph}>
          <FaTimes />
        </div>
      </td>
    </tr>
  )
}

export const Table: FC = () => {
  const dispatch = useDispatch()
  const allGraphs = useSelector(selectAllGraph)

  const updateTopic = useCallback(
    (id: string) =>
      (field: keyof TopicOptions, value: string): void => {
        const feed = allGraphs.find((f) => f.id === id)
        if (!feed) {
          return
        }

        const newGraph: IGraphData = { ...feed.graph }
        switch (field) {
          case 'name':
            newGraph.topic[field] = value
            break
          case 'messageType':
            newGraph.topic[field] = value
            break
        }

        dispatch(
          feedSlice.actions.updateGraph({
            graph: newGraph,
            id,
          })
        )
      },
    [allGraphs, dispatch]
  )

  const updateName = useCallback(
    (id: string) =>
      (value: string): void => {
        const feed = allGraphs.find((f) => f.id === id)
        if (!feed) {
          return
        }
        const newGraph: IGraphData = { ...feed.graph, name: value }
        dispatch(
          feedSlice.actions.updateGraph({
            graph: newGraph,
            id,
          })
        )
      },
    [allGraphs, dispatch]
  )

  return (
    <StyledTable>
      <thead>
        <tr>
          <th>Name</th>
          <th>Topic</th>
          <th>Message Type</th>
          <th />
        </tr>
      </thead>
      <tbody>
        {allGraphs.map((feed) => (
          <TableRow
            key={feed.id}
            feed={feed}
            updateTopic={updateTopic}
            updateName={updateName}
          />
        ))}
      </tbody>
    </StyledTable>
  )
}
