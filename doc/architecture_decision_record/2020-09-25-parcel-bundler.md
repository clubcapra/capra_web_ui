# Parcel bundler

## Decision

We decided to move to parcel for bundling the application and providing a development environment instead of using create-react-app (CRA)

## Status

Accepted

## Context

CRA has multiple issues that became more apparent when moving to electron.

- It lacks configuration and requires ejecting for anything else, which makes it really hard to update
- Is designed for building a front end application only, while it works with electron it isn't an intended use case
- Pollutes the dependency tree with configuration files that can create a conflict. For example, we updated to typescript 4.0.3 which required a newer version of @typescript-eslint. The one included in CRA would require to either eject to update it, or wait on them to update it and they said it wasn't a priority.
- While it worked with bundling the electron side of the application it really wasn't intended to do that. Parcel can independantly bundle the code as long as we give them a root file.
- While it wasn't particularly slow, parcel is still much faster which is nice.

## Consequences

Pros:

- Faster, the caching feature is really nice.
- Can find the root file for the main and renderer simply by reading the package.json.
- Allows for custom configuration, but the default are good enough for most of our use cases.
- Can be updated without ejecting.
- Doesn't pollute the dependencies with configuration that can't be overriden.
- Should correctly bundle images out of the box without specific configuration
- Offers a dev server with hot module reloading by default for react which isn't included in CRA
- Very little amount of configuration required, but still extendable when necessary

Cons:

- Doesn't typecheck the .ts files, it only removes the type annotations. This means we need to at least run tsc manually before building to confirm that the application actually type checks. Vscode still typechecks while developping so it's good enough for our use case which makes this con mostly trivial.
- Less common in the react ecosystem.
- Initial setup involved moving around a lot of things to make it work nicely.
- Required configuring jest manually since it doesn't include a custom test runner, but the config was quite minimal.

## Links

- <https://parceljs.org>
