declare module 'ros3d' {
  import { Ros, TFClient } from 'roslib'

  interface UrdfClientOptions {
    ros: Ros
    param?: string
    tfClient: TFClient
    path?: string
    rootObject?: any
    tfPrefix?: string
    loader?: any
  }

  /**
   * A URDF client can be used to load a URDF and its associated models into a 3D object from the ROS
   * parameter server.
   *
   * Emits the following events:
   *
   * * 'change' - emited after the URDF and its meshes have been loaded into the root object
   *
   * @constructor
   * @param options - object with following keys:
   *
   *   * ros - the ROSLIB.Ros connection handle
   *   * param (optional) - the paramter to load the URDF from, like 'robot_description'
   *   * tfClient - the TF client handle to use
   *   * path (optional) - the base path to the associated Collada models that will be loaded
   *   * rootObject (optional) - the root object to add this marker to
   *   * tfPrefix (optional) - the TF prefix to used for multi-robots
   *   * loader (optional) - the Collada loader to use (e.g., an instance of ROS3D.COLLADA_LOADER)
   */
  class UrdfClient {
    constructor(params: UrdfClientOptions)
  }
}
