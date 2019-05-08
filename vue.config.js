//@ts-check
const ifProduction = (a, b) => (process.env.NODE_ENV === 'production' ? a : b)

const ModuleDependencyWarning = require('webpack/lib/ModuleDependencyWarning')

class IgnoreNotFoundExportPlugin {
  constructor(exportsToIgnore) {
    this.exportsToIgnore = exportsToIgnore || []
  }

  // ↓ Based on https://github.com/sindresorhus/escape-string-regexp
  escapeStringForRegExp(s) {
    return s.replace(/[|\\{}()[\]^$+*?.]/g, '\\$&')
  }

  getMessageRegExp() {
    if (this.exportsToIgnore.length > 0) {
      const exportsPattern =
        '(' +
        this.exportsToIgnore.map(this.escapeStringForRegExp).join('|') +
        ')'

      return new RegExp(
        `export '${exportsPattern}'( \\(reexported as '.*'\\))? was not found in`
      )
    }

    return /export '.*'( \(reexported as '.*'\))? was not found in/
  }

  apply({ hooks, plugin }) {
    const messageRegExp = this.getMessageRegExp()
    const doneHook = stats => {
      stats.compilation.warnings = stats.compilation.warnings.filter(warn => {
        return !(
          warn instanceof ModuleDependencyWarning &&
          messageRegExp.test(warn.message)
        )
      })
    }

    hooks
      ? hooks.done.tap('IgnoreNotFoundExportPlugin', doneHook)
      : plugin('done', doneHook)
  }
}

module.exports = {
  publicPath: ifProduction('/Takin-UI', '/'),
  productionSourceMap: false,
  css: {
    loaderOptions: {
      sass: {
        // Add this to all vue <style>
        // Allows access to all variables and overrides
        data: `@import "@/assets/theme/_variables.scss";`,
      },
    },
  },
  configureWebpack: {
    plugins: [new IgnoreNotFoundExportPlugin()],
  },
}
