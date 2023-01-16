// Theme.tsx
import baseStyled, {
  ThemedStyledInterface,
  ThemedCssFunction,
  css as baseCss,
} from 'styled-components';
import { defaultTheme } from '@/renderer/globalStyles/themes/defaultTheme';

export type Theme = typeof defaultTheme;
export const styled = baseStyled as ThemedStyledInterface<Theme>;
export const css = baseCss as ThemedCssFunction<Theme>;
