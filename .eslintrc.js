const ifProduction = (a, b) => (process.env.NODE_ENV === 'production' ? a : b)

const typescriptConfigs = {
  files: ['*.ts', '*.tsx'],

  parser: '@typescript-eslint/parser',

  // plugins: ['@typescript-eslint'],

  extends: [
    'plugin:@typescript-eslint/recommended',
    'plugin:@typescript-eslint/recommended-requiring-type-checking',
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
    'no-console': [
      ifProduction('error', 'warn'),
      {
        allow: ['warn', 'error'],
      },
    ],
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

  overrides: [typescriptConfigs],

  parserOptions: {
    parser: '@typescript-eslint/parser',
    project: './tsconfig.json',
    ecmaVersion: 2020,
  },

  settings: {
    react: {
      version: 'detect',
    },
  },
}
