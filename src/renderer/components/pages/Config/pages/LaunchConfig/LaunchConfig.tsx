import { Button } from "@/renderer/components/common/Button"
import { feedSlice } from "@/renderer/store/modules/feed"
import { launchFilesSlice, selectAllLaunchFiles } from "@/renderer/store/modules/launchFiles"
import { rosClient } from "@/renderer/utils/ros/rosClient"
import React, { FC } from "react"
import { useDispatch, useSelector } from "react-redux"
import ROSLIB from "roslib"
import { LaunchElement } from "./LaunchElement"

export const LaunchConfig : FC = () => {
    const dispatch = useDispatch();

    const onClick = (fileName: string) => {
        dispatch(launchFilesSlice.actions.launchFile(fileName));
        launchHanlderTopic.publish(fileName);
    }

    const onClickAll = () => {
        allLaunchFiles.forEach(element => {
            onClick(element.fileName);
        });
    }

    const allLaunchFiles = useSelector(selectAllLaunchFiles);

    return(
        <>
        <Button onClick={onClickAll}>Launch All</Button>
        {allLaunchFiles.map(element => <LaunchElement key={element.name} name={element.name} launchFile={element.fileName} onClick={onClick} isLaunched={element.isLaunched}></LaunchElement>)}
        </>
    )

}

const launchHanlderTopic = new ROSLIB.Topic({
    ros: rosClient.ros,
    name: "launchHandler/",
    messageType: "std_msgs/String"
});
