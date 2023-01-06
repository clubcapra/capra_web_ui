import type { preload } from '@/main/preload/api';

declare global {
  interface Window {
    preloadApi: typeof preload;
  }
}
