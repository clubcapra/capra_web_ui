import React, { FC } from 'react'
import { useDispatch } from 'react-redux'
import { feedSlice } from '@/renderer/store/modules/feed'
import { Table } from './Table'
import { Button } from '@/renderer/components/common/Button'
import { SectionTitle } from '@/renderer/components/pages/Config/styles'
import { styled } from '@/renderer/globalStyles/styled'

const AddGraph = () => {
  const dispatch = useDispatch()

  const addGraph = () =>
    dispatch(
      feedSlice.actions.addGraph({
        name: '',
        topic: {
          name: '',
          messageType: '',
        },
      })
    )

  return <Button onClick={addGraph}>Add Graph Data Source</Button>
}

const TableSection = () => (
  <>
    <SectionTitle>Graphs</SectionTitle>
    <AddGraph />
    <Table />
  </>
)

const GraphConfigWrapper = styled.div`
  width: 100%;
  height: 100%;
`

export const GraphConfig: FC = () => {
  return (
    <GraphConfigWrapper>
      <SectionTitle>Notice</SectionTitle>
      <p>
        This assumes that the data can be converted to a number. This also
        assumes that the topic receives a single message for each new graph
        value.
      </p>
      <TableSection />
    </GraphConfigWrapper>
  )
}
