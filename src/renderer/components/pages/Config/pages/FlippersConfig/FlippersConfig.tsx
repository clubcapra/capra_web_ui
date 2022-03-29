import React, { FC } from 'react'
import { Table } from '../FlippersConfig/Table'
import { SectionTitle } from '@/renderer/components/pages/Config/styles'
import { styled } from '@/renderer/globalStyles/styled'

const TableSection = () => (
  <>
    <SectionTitle>Flippers</SectionTitle>
    <Table />
  </>
)

const FlippersConfigWrapper = styled.div`
  width: 100%;
  height: 100%;
`

export const FlippersConfig: FC = () => {
  return (
    <FlippersConfigWrapper>
      <SectionTitle>Notice</SectionTitle>
      <p>This assumes that the data can be converted to a number</p>
      <TableSection />
    </FlippersConfigWrapper>
  )
}
