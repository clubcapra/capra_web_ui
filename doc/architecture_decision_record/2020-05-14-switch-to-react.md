# Switch to react

## Decision

Switch to react from vue for the view layer.

## Status

Accepted

## Context

<!-- What is the context of this decision? It is important to capture the full context of the decision so that the reader knows the reasons behind it. -->

After the first competition with the v1.0 of the UI there was a lot of new features necessary. The team had more experience with react than vue. There was already a rewrite done in react for a user interface class with no behaviour. Vue 3.0 was announced but it was still far from being ready. The team wants to use Typescript, but vue 2.0 with typescript wasn't great compared to react.

## Consequences

<!-- In this section, you can add what would happen if this decision is made. It is important to list all consequences, both positive and negative. -->
Pros:

- Better typescript support
- Hook style programming (was only non-official support for vue at the time)
- More experience with react in the team

Cons:

- Major rewrite
