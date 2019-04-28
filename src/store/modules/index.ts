/**
 * The file enables `@/store/index.ts` to import all vuex modules
 * in a one-shot manner. There should not be any reason to edit this file.
 */

const files = require.context('.', false, /\.ts$/)
const modules: any = {}

files.keys().forEach(key => {
  if (key === './index.ts') return
  modules[key.replace(/(\.\/|\.ts)/g, '')] = files(key).default
})

const jsFiles = require.context('.', false, /\.js$/)
jsFiles.keys().forEach(key => {
  modules[key.replace(/(\.\/|\.js)/g, '')] = jsFiles(key).default
})

export default modules
