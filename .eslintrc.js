const ifProduction = (a, b) => (process.env.NODE_ENV === 'production' ? a : b)

module.exports = {
  root: true,

  env: {
    node: true,
  },

  extends: ['plugin:vue/recommended', '@vue/prettier', '@vue/typescript'],

  rules: {
    'no-console': [
      'off',
      {
        allow: ['warn', 'error'],
      },
    ],
    'no-debugger': 'off',
    'no-var': 'error',
    'prettier/prettier': 'warn',
    'vue/no-unused-components': 'warn',
    'vue/html-self-closing': [
      'error',
      {
        html: {
          void: 'any',
        },
      },
    ],
    'prefer-const': [
      'error',
      {
        destructuring: 'any',
        ignoreReadBeforeAssign: false,
      },
    ],
    'linebreak-style': ['error', 'windows'],
  },

  parserOptions: {
    parser: '@typescript-eslint/parser',
  },
}
