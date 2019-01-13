const production = (a, b) => (process.env.NODE_ENV === 'production' ? a : b)

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
  plugins: ['vue', 'prettier'],
  extends: [
    'plugin:vue/recommended',
    'prettier',
    'prettier/vue',
    'eslint:recommended'
  ],
  rules: {
    'no-console': production('error', 'off'),
    'no-debugger': production('error', 'off'),
    'no-var': 'error',
    'no-unused-vars': production('error', 'warn'),
    'prettier/prettier': 'warn',
    'vue/no-unused-components': production('error', 'warn'),
    'vue/max-attributes-per-line': 'off',
    'vue/html-self-closing': [
      'error',
      {
        html: {
          void: 'any'
        }
      }
    ],
    'prefer-const': [
      'error',
      {
        destructuring: 'any',
        ignoreReadBeforeAssign: false
      }
    ]
  }
}
