const ifProduction = (a, b) => (process.env.NODE_ENV === 'production' ? a : b)

module.exports = {
  root: true,
  env: {
    node: true,
  },
  extends: ['plugin:vue/essential', '@vue/prettier', '@vue/typescript'],
  rules: {
    'no-console': [ifProduction('error', 'off'), { allow: ['warn', 'error'] }],
    'no-debugger': ifProduction('error', 'off'),
    'no-var': 'error',
    //'no-unused-vars': ifProduction('error', 'warn'),
    'prettier/prettier': 'warn',
    'vue/no-unused-components': ifProduction('error', 'warn'),
    'vue/max-attributes-per-line': 'off',
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
  },
  parserOptions: {
    parser: '@typescript-eslint/parser',
  },
}
