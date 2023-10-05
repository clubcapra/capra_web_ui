import { fileURLToPath, URL } from 'node:url';

import vue from '@vitejs/plugin-vue';
import { defineConfig, type UserConfig } from 'vite';
import { checker } from 'vite-plugin-checker';
import vuetify, { transformAssetUrls } from 'vite-plugin-vuetify';
import { configDefaults } from 'vitest/config';

/**
 * Vite Configure
 *
 * @see {@link https://vitejs.dev/config/}
 */
export default defineConfig(
  async (): Promise<UserConfig> => ({
    plugins: [
      // Vue3
      vue({
        template: {
          // https://github.com/vuetifyjs/vuetify-loader/tree/next/packages/vite-plugin#image-loading
          transformAssetUrls,
        },
      }),
      // Vuetify Loader
      // https://github.com/vuetifyjs/vuetify-loader/tree/next/packages/vite-plugin#vite-plugin-vuetify
      vuetify({
        autoImport: true,
        styles: { configFile: 'src/styles/settings.scss' },
      }),
      // vite-plugin-checker
      // https://github.com/fi3ework/vite-plugin-checker
      checker({
        typescript: true,
        vueTsc: true,
        eslint: {
          lintCommand:
            'eslint . --ext .vue,.js,.jsx,.cjs,.mjs --cache --cache-location ./node_modules/.vite/vite-plugin-eslint --ignore-path .eslintignore', // for example, lint .ts & .tsx
        },
      }),
    ],
    // prevent vite from obscuring rust errors
    // https://vitejs.dev/config/shared-options.html#clearscreen
    clearScreen: false,
    // https://vitejs.dev/config/server-options.html
    server: {
      port: 1420,
      strictPort: true,
    },
    // Resolver
    resolve: {
      // https://vitejs.dev/config/shared-options.html#resolve-alias
      alias: {
        '@': fileURLToPath(new URL('./src', import.meta.url)),
      },
      extensions: ['.js', '.json', '.jsx', '.mjs', '.ts', '.tsx', '.vue'],
    },
    // https://tauri.studio/v1/api/config#buildconfig.beforedevcommand
    envPrefix: ['VITE_', 'TAURI_'],
    test: {
      // https://vitest.dev/guide/#configuring-vitest
      clearMocks: true,
      environment: 'node',
      exclude: [...configDefaults.exclude, 'e2e/*', 'src-tauri/*'],
      globals: true,
      globalSetup: [
        fileURLToPath(new URL('./vitest/setup.ts', import.meta.url)),
      ],
      root: fileURLToPath(new URL('./', import.meta.url)),
    },
  })
);
