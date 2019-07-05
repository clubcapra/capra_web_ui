import { ColorHSL, ColorRGB } from './types'
import _ from 'lodash-es'

export function interpolateRGB(
  color1: ColorRGB,
  color2: ColorRGB,
  factor: number
): ColorRGB {
  const result: ColorRGB = { ...color1 }

  return _.mapValues<ColorRGB, number>(result, (value: number, k: string) => {
    return Math.round(value + factor * (color2[k] - color1[k]))
  })
}

export function hexToRgb(hex: string): ColorRGB {
  hex = hex.replace(/[^0-9A-F]/gi, '')

  const hexValue = parseInt(hex, 16)
  const r = (hexValue >> 16) & 255
  const g = (hexValue >> 8) & 255
  const b = hexValue & 255

  return { r, g, b }
}
