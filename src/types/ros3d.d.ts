// Type definitions for ros3d
// Definitions by: Charles Giguere <https://github.com/IceSentry>
// Definitions: https://github.com/DefinitelyTyped/DefinitelyTyped

declare module 'ros3d' {
  import { Ros, TFClient } from 'roslib'
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
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  class UrdfClient {
    constructor(options: {
      ros: Ros
      param?: string
      tfClient: TFClient
      path?: string
      rootObject?: unknown
      tfPrefix?: string
      loader?: unknown
    })
  }
  /**
   * A Viewer can be used to render an interactive 3D scene to a HTML5 canvas.
   *
   * @constructor
   * @param options - object with following keys:
   *
   *  * divID - the ID of the div to place the viewer in
   *  * width - the initial width, in pixels, of the canvas
   *  * height - the initial height, in pixels, of the canvas
   *  * background (optional) - the color to render the background, like '#efefef'
   *  * alpha (optional) - the alpha of the background
   *  * antialias (optional) - if antialiasing should be used
   *  * intensity (optional) - the lighting intensity setting to use
   *  * cameraPosition (optional) - the starting position of the camera
   *  * displayPanAndZoomFrame (optional) - whether to display a frame when
   *  *                                     panning/zooming. Defaults to true.
   *  * lineTypePanAndZoomFrame - line type for the frame that is displayed when
   *  *                           panning/zooming. Only has effect when
   *  *                           displayPanAndZoomFrame is set to true.
   */
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  class Viewer {
    scene: unknown
    constructor(params: {
      divID: string
      width: number
      height: number
      background?: string
      antialias?: boolean
      intensity?: number
      near?: number
      far?: number
      alpha?: number
      cameraPose?: {
        x: number
        y: number
        z: number
      }
      cameraZoomSpeed?: number
      displayPanAndZoomFrame?: boolean
      lineTypePanAndZoomFrame?: string
    })
    /**
     *  Start the render loop
     */
    start(): void
    /**
     *  Stop the render loop
     */
    stop(): void
    /**
     * Renders the associated scene to the viewer.
     */
    draw(): void
    /**
     * Add the given THREE Object3D to the global scene in the viewer.
     *
     * @param object - the THREE Object3D to add
     * @param selectable (optional) - if the object should be added to the selectable list
     */
    addObject(object: object, selectable?: boolean): void
    /**
     * Resize 3D viewer
     *
     * @param width - new width value
     * @param height - new height value
     */
    resize(width: number, height: number): void
  }
  /**
   * Create a grid object.
   *
   * @constructor
   * @param options - object with following keys:
   *
   *  * num_cells (optional) - The number of cells of the grid
   *  * color (optional) - the line color of the grid, like '#cccccc'
   *  * lineWidth (optional) - the width of the lines in the grid
   *  * cellSize (optional) - The length, in meters, of the side of each cell
   */
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  class Grid {
    constructor(options?: {
      num_cells?: number
      color?: string
      lineWidth?: number
      cellSize?: number
    })
  }
  /**
   * A MeshResource is an THREE object that will load from a external mesh file. Currently loads
   * Collada files.
   *
   * @constructor
   * @param options - object with following keys:
   *
   *  * path (optional) - the base path to the associated models that will be loaded
   *  * resource - the resource file name to load
   *  * material (optional) - the material to use for the object
   *  * warnings (optional) - if warnings should be printed
   */
  // eslint-disable-next-line @typescript-eslint/no-unused-vars
  class MeshResource {
    constructor(options: {
      path?: string
      resource: string
      material?: string
      warnings?: boolean
    })
  }
}
