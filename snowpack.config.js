module.exports = {
  extends: 'electron-snowpack/config/snowpack.js',
  mount: {
    'src/shared': '/shared',
  },
  alias: {
    '@/': './src/',
  },
  devOptions: {
    port: Number.parseInt(process.env.ELECTRON_SNOWPACK_PORT, 10),
  },
  packageOptions: {
    knownEntrypoints: ['date-fns/fp/format'],
  },
  plugins: ['@snowpack/plugin-react-refresh'],
}
