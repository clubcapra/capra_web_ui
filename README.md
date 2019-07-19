# takin-ui

User interface for the Takin project. It can be used for any similar robots with the same set of feature using ROS.

## hosted on github pages

<https://www.clubcapra.com/Takin-UI/>

## Technologies used

- Typescript
- Vue.js
  - vuex
    - vuex-class-component
    - vuex-persist
  - vue-router
  - vue-class-component
  - vue-property-decorator
  - vue-cli
- yarn
- css grid, sass/scss
- bulma
  - vue-bulma-components to integrate with vue
- eslint
- prettier
- webpack
- roslibjs
- lodash
- font-awesome

You can look in project.json for more specific dependencies.

**Vue is the most important dependency here and you should know it to work on this project.**

You do not need to be familiar with everything on this list but you should try using these before considering adding a new dependency. We try not to add dependency for simple functionalities, try to either write it yourself or use already used dependencies. lodash provides a lot of utility functions not provided by the small javascript standard library.

Note: most javascript projects uses npm. While we do not use it directly you should already have it installed. For package management and build scripts we use yarn.

## Folder structure

src is essentially the root folder everything else is configuration files that shouldn't be touched unless necessary.

- **src**:
  - **assets**: contains any artifacts that aren't code. For examples: images or theme files that are loaded but not modified.
  - **components**: contains all vue components used to build the application. The internal structure isn't defined for now, but we try to sort them by features.
  - **store**: contains vuex modules and other utility functions related to using these stores
  - **utils**:
    - **gamepad**: wrapper to simplify using the gamepad api and also does the input handling
    - **math**: math stuff like Vecor2/Vector3
    - **ros**: RosClient is a wrapper for the roslibjs library to simplify usage
  - **App.vue**: main entry point of the vue app
  - **main.ts**: main entry point of the entire web app. This should rarely change.
  - **router.ts**: Since our router is really simple we only have a single file. If it becomes more complex we will make a router folder

Vue is a component based framework, this means we try to use composition as much as possible. Since this is a vue app we try to use vue features and the vue way of doing things.

## Build/run Setup

### if yarn is not installed

```bash
npm install -g yarn
```

### Project setup

```bash
yarn install
```

### Gives a nice ui to launch all these commands and also other goodies like analyzing build and dependeny management

needs `yarn global add @vue/cli@next`

```bash
vue ui
```

### Compiles and hot-reloads for development

```bash
yarn serve
```

### Compiles and minifies for production

```bash
yarn build
```

### Lints and fixes files

```bash
yarn lint
```

## ROS dependencies

rosbrige_suite
web_video_server

Use `takin_web_ui.launch` in https://github.com/clubcapra/takin_bringup/tree/master/launch to launch necessayr ros dependencies

### Want a new feature or a bug fixed?

Create an issue and it will automatically be added to the project board and be tracked there
