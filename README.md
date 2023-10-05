# capra_web_ui

This project is an application with a user interface (UI) designed for manually controlling ROS-based robots in real-time. While it is primarily intended for use in the context of the Capra club, we have aimed to make it as robot-agnostic as possible. As long as your robots have the required dependencies, installation should be nearly plug and play.

- [capra\_web\_ui](#capra_web_ui)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Running the Web Application](#running-the-web-application)
  - [Running the Desktop Application with Tauri](#running-the-desktop-application-with-tauri)
  - [Building the Application with Tauri](#building-the-application-with-tauri)
  - [Release](#release)
  - [Recommended IDE Setup](#recommended-ide-setup)
  - [Type Support For `.vue` Imports in TS](#type-support-for-vue-imports-in-ts)

## Prerequisites

Before you can run the application, make sure you have the following software installed on your system:

- [Node.js and npm](https://nodejs.org/) - To run the web application.
- [Rust](https://www.rust-lang.org/) - To run the desktop application with Tauri.

## Installation

Follow these steps to install the prerequisites:

1. Install Node.js and npm by following the instructions available on the [official Node.js website](https://nodejs.org/).
2. Install Rust by following the instructions available on the [official Rust website](https://www.rust-lang.org/).

Once you have installed these software, you are ready to run the application.

> **Note for Windows Users:** While you can use WSL (Windows Subsystem for Linux) to install and run the required tools, we recommend using the native Windows versions. WSL can significantly slow down your development process, making your code execution slower than it should be.

## Running the Web Application

To run the web application, open a terminal and follow these steps:

1. Navigate to the project directory:
   ```shell
   cd capra_web_ui
   ```
2. Install npm dependenccies by running the following command:
   ```shell
   npm install
   ```
3. Launch the web application using Vite:
   ```shell
   npm run dev
   ```

The web application will be accessible at <http://localhost:1420>.

## Running the Desktop Application with Tauri

To run the desktop application with Tauri, open a terminal and follow these steps:

1. Navigate to the project directory:
   ```shell
   cd capra_web_ui
   ```
2. Make sure npm dependencies are installed by running:
   ```shell
   npm install
   ```
3. Launch the desktop application using the Tauri command:
   ```shell
   npm run tauri dev
   ```

The desktop application will be launched using Tauri.

## Building the Application with Tauri

To build the desktop application with Tauri, follow these steps:

1. Navigate to the project directory:
   ```shell
   cd capra_web_ui
   ```
2. Install npm dependencies by running the following command:
   ```shell
   npm install
   ```
3. Build. the desktop application with Tauri:
    ```shell
    npm run tauri build
    ```

## Release

To create a new release, simply use `npm version [major | minor | patch]`. This will bump the version and create a git tag. You can then push the new commit and github actions will take care of everythin else. You can use `git push --follow-tags` to push the tags to github.

Just make sure to make the github release public once it's done.

## Recommended IDE Setup

- [VS Code](https://code.visualstudio.com/) + [Volar](https://marketplace.visualstudio.com/items?itemName=Vue.volar) (and disable Vetur) + [TypeScript Vue Plugin (Volar)](https://marketplace.visualstudio.com/items?itemName=Vue.vscode-typescript-vue-plugin) + [Tauri](https://marketplace.visualstudio.com/items?itemName=tauri-apps.tauri-vscode) + [rust-analyzer](https://marketplace.visualstudio.com/items?itemName=rust-lang.rust-analyzer).

## Type Support For `.vue` Imports in TS

TypeScript cannot handle type information for `.vue` imports by default, so we replace the `tsc` CLI with `vue-tsc` for type checking. In editors, we need [TypeScript Vue Plugin (Volar)](https://marketplace.visualstudio.com/items?itemName=Vue.vscode-typescript-vue-plugin) to make the TypeScript language service aware of `.vue` types.

If the standalone TypeScript plugin doesn't feel fast enough to you, Volar has also implemented a [Take Over Mode](https://github.com/johnsoncodehk/volar/discussions/471#discussioncomment-1361669) that is more performant. You can enable it by the following steps:

1. Disable the built-in TypeScript Extension
   1. Run `Extensions: Show Built-in Extensions` from VSCode's command palette
   2. Find `TypeScript and JavaScript Language Features`, right click and select `Disable (Workspace)`
2. Reload the VSCode window by running `Developer: Reload Window` from the command palette.
