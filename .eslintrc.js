const typescriptConfigs = {
  files: ['*.ts', '*.tsx'],

  parserOptions: {
    project: './tsconfig.json',
    ecmaVersion: 2020,
  },

  extends: [
    'plugin:@typescript-eslint/recommended',
    'plugin:@typescript-eslint/recommended-requiring-type-checking',
    'plugin:react-hooks/recommended',
  ],

  rules: {
    '@typescript-eslint/explicit-member-accessibility': [
      1,
      { accessibility: 'no-public' },
    ],
    '@typescript-eslint/explicit-function-return-type': [
      0,
      { allowExpressions: true, allowTypedFunctionExpressions: true },
    ],
    '@typescript-eslint/interface-name-prefix': 0,
    '@typescript-eslint/camelcase': 0,
    '@typescript-eslint/explicit-module-boundary-types': 0,
    '@typescript-eslint/restrict-template-expressions': 0,
    '@typescript-eslint/no-misused-promises': [
      'error',
      {
        checksVoidReturn: {
          returns: false,
        },
      },
    ],
  },
}

module.exports = {
  root: true,

  env: {
    node: true,
  },

  extends: [
    'eslint:recommended',
    'plugin:react/recommended',
    'plugin:prettier/recommended',
  ],

  rules: {
    'no-console': 'error',
    'no-debugger': 'off',
    'no-var': 'error',
    'prefer-const': [
      'error',
      {
        destructuring: 'any',
        ignoreReadBeforeAssign: false,
      },
    ],
    'linebreak-style': ['error', 'windows'],
    'react/self-closing-comp': 1,
    'react/prop-types': 0,
    curly: 'error',
  },

  parser: '@typescript-eslint/parser',

  overrides: [typescriptConfigs],

  settings: {
    react: {
      version: 'detect',
    },
  },
}
