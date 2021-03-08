// Theme.tsx
import baseStyled, { ThemedStyledInterface } from 'styled-components'
import { defaultTheme } from '@/renderer/globalStyles/themes/defaultTheme'

export type Theme = typeof defaultTheme
export const styled = baseStyled as ThemedStyledInterface<Theme>
