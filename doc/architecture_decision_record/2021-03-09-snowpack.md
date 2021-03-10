# Parcel bundler

## Decision

We decided to move to snowpack for bundling the application and to provide a development environment instead of using parcel

## Status

Accepted

## Context

Parcel v2 was supposed to be right around the corner last year, but it's still not released. Parcel v1 was still simpler and faster than CRA, but after trying snowpack it was even faster and simpler to configure

## Consequences

Pros:

- Really fast
- Uses modern web standards
- electron-snowpack makes integration with electron simple
- Doesn't require ejecting
- Intergrated devserver with HMR and other plugins
- Simplified the build steps

Cons:

- There was an issue with a dependency of roslib that made it hard to run through snowpack. Fortunately the fix was quite simple and only required to patch a small part of the lib to make it work
- Still another change in bundler
- Still fairly new

## Links

- <https://www.snowpack.dev>
- <https://github.com/karolis-sh/electron-snowpack>
