const ifProduction = (a, b) => (process.env.NODE_ENV === 'production' ? a : b)

module.exports = {
  root: true,

  env: {
    node: true,
  },

  extends: [
    'react-app',
    'eslint:recommended',
    'plugin:@typescript-eslint/eslint-recommended',
    'plugin:@typescript-eslint/recommended',
    'prettier',
    'prettier/@typescript-eslint',
    'plugin:react/recommended',
  ],

  plugins: ['prettier', '@typescript-eslint', 'react'],

  rules: {
    'no-console': [
      ifProduction('error', 'warn'),
      {
        allow: ['warn', 'error'],
      },
    ],
    'no-debugger': 'off',
    'no-var': 'error',
    'prettier/prettier': 'warn',
    'prefer-const': [
      'error',
      {
        destructuring: 'any',
        ignoreReadBeforeAssign: false,
      },
    ],
    'linebreak-style': ['error', 'windows'],
    '@typescript-eslint/explicit-member-accessibility': [
      1,
      { accessibility: 'no-public' },
    ],
    '@typescript-eslint/explicit-function-return-type': [
      1,
      { allowExpressions: true, allowTypedFunctionExpressions: true },
    ],
    '@typescript-eslint/interface-name-prefix': 0,
    '@typescript-eslint/camelcase': 0,
    'react/self-closing-comp': 1,
    'react/prop-types': 0,
  },

  parserOptions: {
    parser: '@typescript-eslint/parser',
    project: './tsconfig.json',
  },
}
