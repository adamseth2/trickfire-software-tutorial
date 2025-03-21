import { fileURLToPath, URL } from 'node:url';
import { defineConfig } from 'vite';
import { templateCompilerOptions } from '@tresjs/core';
import vue from '@vitejs/plugin-vue';
import vueDevTools from 'vite-plugin-vue-devtools';

const useDevTools = process.env.USE_VITE_DEVTOOLS === 'true';
const plugin = useDevTools ? [vueDevTools()] : [];
// https://vitejs.dev/config/

// vite.config.ts has to be default export
// eslint-disable-next-line no-restricted-exports
export default defineConfig({
  plugins: [vue({ ...templateCompilerOptions }), ...plugin],
  server: {
    // automatically opens in browser
    open: true,
    port: 8000,
  },
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
    },
  },
  // Fixes The legacy JS API is deprecated and will be removed in Dart Sass 2.0.0 when updating packages
  css: {
    preprocessorOptions: {
      scss: {
        api: 'modern-compiler', // or "modern",
        // Silence issue
        silenceDeprecations: ['legacy-js-api'],
      },
    },
  },
  base: process.env.GH_PAGES === 'true' ? '/trickfire-software-tutorial/' : '/',
});
