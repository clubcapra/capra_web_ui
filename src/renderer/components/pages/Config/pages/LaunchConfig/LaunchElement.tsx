import { Button } from "@/renderer/components/common/Button"
import React, { FC, useState } from "react"

interface LaunchElementProps {
    name: string,
    launchFile: string,
    onClick: Function,
    isLaunched?: boolean
}

export const LaunchElement: FC<LaunchElementProps>= (props: LaunchElementProps) => {

    var stylingObject = {
        div: {
          display: "flex",
          marginTop: 5
        },
        h3: {
            flexGrow : 1,
            width: "75%"
        }
      }

    return (
        <>
            <div style={stylingObject.div}>
                <h3 style={stylingObject.h3}>{props.name}</h3> 
                <Button onClick={() => props.onClick(props.launchFile)} btnType={props.isLaunched ? "danger" : "success"}>{props.isLaunched ? "Kill" : "Launch"}</Button>
            </div>
            
        </>
    )
}

