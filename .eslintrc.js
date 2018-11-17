module.exports = {
  root: true,
  parserOptions: {
    parser: 'babel-eslint',
    sourceType: 'module'
  },
  env: {
    browser: true,
    node: true
  },
  globals: {
    __static: true
  },
  plugins:['vue', 'prettier'],
  extends: ['prettier', 'plugin:vue/recommended'],
  rules: {
    'no-console': process.env.NODE_ENV === 'production' ? 'error' : 'off',
    'no-debugger': process.env.NODE_ENV === 'production' ? 'error' : 'off',
    'no-var': 'error',
    "prettier/prettier": "error",
    'vue/max-attributes-per-line': "off",
    "vue/html-self-closing": [
      "error",
      {
        "html": {
          "void": "any"
        }
      }
    ],
    "prefer-const": ["error", {
      "destructuring": "any",
      "ignoreReadBeforeAssign": false
  }]
  }
}
