import { LocationState } from 'history'
import { Context, useContext, useEffect } from 'react'
import {
  __RouterContext,
  RouteComponentProps,
  StaticContext,
} from 'react-router'
import useForceUpdate from 'use-force-update'

type AnyContext = Context<RouteComponentProps<any, any, any>>

export function useRouter<
  P extends { [K in keyof P]?: string } = {},
  C extends StaticContext = StaticContext,
  S = LocationState
>(): RouteComponentProps<P, C, S> {
  const context: RouteComponentProps<P, C, S> = useContext<
    RouteComponentProps<P, C, S>
  >((__RouterContext as AnyContext) as Context<RouteComponentProps<P, C, S>>)

  const forceUpdate = useForceUpdate()
  useEffect(() => context.history.listen(forceUpdate), [context, forceUpdate])
  return context
}
