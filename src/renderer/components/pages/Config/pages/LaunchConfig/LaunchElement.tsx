import { Button } from "@/renderer/components/common/Button"
import React, { FC, useState } from "react"

interface LaunchElementProps {
    name: string,
    launchFile: string,
    onClick: Function,
    isLaunched?: boolean
}

export const LaunchElement: FC<LaunchElementProps>= (props: LaunchElementProps) => {

    return (
        <>
            <p>{props.name}</p> <Button onClick={() => props.onClick(props.launchFile)} btnType={props.isLaunched ? "danger" : "success"}>{props.isLaunched ? "Kill" : "Launch"}</Button>
        </>
    )
}

