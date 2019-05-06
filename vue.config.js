module.exports = {
  publicPath: '/',
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
}
