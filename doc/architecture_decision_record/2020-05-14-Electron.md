# Electron

## Decision

Port the existing application to electron

## Status

Accepted

## Context

<!-- What is the context of this decision? It is important to capture the full context of the decision so that the reader knows the reasons behind it. -->

There was difficulty with hosting the website with github pages since it is not possible to have multiple branches on a single gh page. We needed an easier way to bundle the entire app that didn't require an internet connection. PWA were investigated, but the ease of disctirbution of an electron app for our use case was better.

## Consequences

<!-- In this section, you can add what would happen if this decision is made. It is important to list all consequences, both positive and negative. -->

Pros:

- Easy to bundle and distribute multiple versions
- Easy to port since it's web technologies already
- Support for node and directly accessing the host machine if necessary
- Controlled version of the browser since we have some dependencies that don't work in some browsers

Cons:

- Potential performance issues, but the application is the only process on the host machine so it doesn't really matter for our use case
- No easy way to let anyone use it directly in the browser, but again not our main use case
