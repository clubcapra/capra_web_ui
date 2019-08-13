# capra-web-ui

Web based user interface for ROS based robots.

## hosted on github pages

<https://www.clubcapra.com/capra_web_ui/>

## Technologies used

- Typescript
- React
- css grid
- eslint
- prettier
- webpack
- roslibjs
- lodash
- font-awesome

You can look in project.json for more specific dependencies.

## Folder structure

src is essentially the root folder everything else is configuration files that shouldn't be touched unless necessary.

- **src**:
  - **assets**: contains any artifacts that aren't code. For examples: images or theme files that are loaded but not modified.
  - **components**: contains all the components used to build the application. The internal structure isn't defined for now, but we try to sort them by features.
  - **store**: contains vuex modules and other utility functions related to using these stores
  - **utils**:
    - **gamepad**: wrapper to simplify using the gamepad api and also does the input handling
    - **math**: math stuff like Vecor2/Vector3
    - **ros**: RosClient is a wrapper for the roslibjs library to simplify usage
  - **App.tsx**: main entry point of the vue app
  - **index.ts**: main entry point of the entire web app. This should rarely change.

React is a component based framework, this means we try to use composition as much as possible. Since this is a react app we try to use react features and the react way of doing things.

### Project setup

See the section about [deployment](https://facebook.github.io/create-react-app/docs/deployment) for more information.

### `npm run eject`

**Note: this is a one-way operation. Once you `eject`, you can’t go back!**

If you aren’t satisfied with the build tool and configuration choices, you can `eject` at any time. This command will remove the single build dependency from your project.

Instead, it will copy all the configuration files and the transitive dependencies (Webpack, Babel, ESLint, etc) right into your project so you have full control over them. All of the commands except `eject` will still work, but they will point to the copied scripts so you can tweak them. At this point you’re on your own.

You don’t have to ever use `eject`. The curated feature set is suitable for small and middle deployments, and you shouldn’t feel obligated to use this feature. However we understand that this tool wouldn’t be useful if you couldn’t customize it when you are ready for it.

## Learn More

You can learn more in the [Create React App documentation](https://facebook.github.io/create-react-app/docs/getting-started).

To learn React, check out the [React documentation](https://reactjs.org/).

## ROS dependencies

rosbrige_suite
web_video_server

Use `takin_web_ui.launch` in https://github.com/clubcapra/takin_bringup/tree/master/launch to launch necessary ros dependencies

TODO change the launch file
