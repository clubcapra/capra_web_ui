import type { preload } from '@/main/preload'

declare global {
  interface Window {
    preloadApi: typeof preload
  }
}
