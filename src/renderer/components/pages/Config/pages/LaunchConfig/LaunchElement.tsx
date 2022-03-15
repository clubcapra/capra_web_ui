import { Button } from '@/renderer/components/common/Button'
import React, { FC, useState } from 'react'
import ClipLoader from 'react-spinners/ClipLoader'

interface LaunchElementProps {
  name: string
  launchFile: string
  packageName: string
  onClick: (fileName: string, packageName: string) => void
  isLaunched?: boolean
}

export const LaunchElement: FC<LaunchElementProps> = (
  props: LaunchElementProps
) => {
  const stylingObject = {
    div: {
      display: 'flex',
      marginTop: 5,
    },
    h3: {
      flexGrow: 1,
      width: '75%',
    },
  }

  return (
    <>
      <div style={stylingObject.div}>
        <h3 style={stylingObject.h3}>{props.name}</h3>
        <Button
          onClick={() => props.onClick(props.launchFile, props.packageName)}
          btnType={props.isLaunched ? 'danger' : 'success'}
        >
          {props.isLaunched ? 'Kill' : 'Launch'}
        </Button>
      </div>
    </>
  )
}
