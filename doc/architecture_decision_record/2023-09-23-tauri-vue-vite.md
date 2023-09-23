# Title

Migration to Tauri, Vue 3, and Vite

## Decision

The decision has been made to migrate our application from React to Vue.js, from Electron to Tauri, and from Snowpack to Vite.

## Status

Proposed

## Context

### React -> Vue 3

The Vue 3 framework has undergone a complete rewrite and is now believed to be more performant than React.
Additionally, a lot of new features were added:

- Composition API: a new way to write components that is more flexible and powerful than the Options API.
  - Better Logic Reuse: It enables clean and efficient logic reuse. Allow us to extract common logic into reusable functions and reuse them across components.
  - More Flexible Code Organization: It allows us to organize our code in a more flexible way. We can group logic by feature instead of by lifecycle hooks.
  - Better Type Inference: Composition API utilizes mostly plain variables and functions, which are naturally type friendly.
- SFC Composition API Syntax Sugar (`<script setup>`)
  - Smaller Production Bundle and Less Overhead: Code written in Composition API and `<script setup>` is more efficient and minification-friendly.
    - More succinct code with less boilerplate
    - Ability to declare props and emitted events using pure TypeScript
    - Better runtime performance (the template is compiled into a render function in the same scope, without an intermediate proxy)
    - Better IDE type-inference performance (less work for the language server to extract types from code)
- Teleport: built-in component that allow to "teleport" a part of a component's template into a DOM node that exists outside the DOM hierarchy of that component.
- Fragments; i.e., multiple root nodes

Another factors:

- Vue 3 offers a more intuitive environment for beginners, which can encourage more students to get involved in our club.
- Vue 3 is considered to be lighter and less complex than React, which can reduce the learning curve for our development team and speed up the development process.

## Electron -> Tauri

Tauri is a framework for building native apps with web technologies. It is a lightweight alternative to Electron.

Tauri offers better possibilities for directly handling video streams compared to Electorn, without the latencies associated with existing integration solutions. This will enhance the user experience of our application.

## Snowpack -> Vite

Vite is a build tool that is designed to be fast and stable. It is renowned for its speed and stability in the build process, which can increase our work speed and improve development efficiency.

## Consequences

### Pros:

- **Improved code maintainability**: Migration to Vue.js, Tauri, and Vite should make our code easier to maintain, which will reduce the time and effort required to fix bugs and add new features.
- **Resolution of video streaming issues**: This migration should resolve video streaming issues and enhance the quality of the user experience.
- **Increased stability and speed of the build system**: Vite will provide better stability and greater speed in the build process, accelerating the development cycle.
- **More welcoming environment for students**: The use of well-regarded technologies like Vue.js can attract more students to our programming club, strengthening our community and resources.
- **Simplified development of new features**: Vue.js's structure will make adding new features easier, enabling faster response to user needs.

### Cons:

- **Major rewrite**: This migration will require a major rewrite of our codebase, which will take time and effort.
- **Learning curve**: The migration will require our team to learn new technologies, which will take time and effort.
- **Potential for new bugs**: The migration may introduce new bugs into our codebase, which will take time and effort to fix.
