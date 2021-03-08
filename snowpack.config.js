module.exports = {
  extends: 'electron-snowpack/config/snowpack.js',
  mount: {
    'src/shared': '/shared',
  },
  alias: {
    '@/': './src/',
  },
  devOptions: {
    port: 7918,
  },
  packageOptions: {
    knownEntrypoints: ['date-fns/fp/format'],
  },
}
