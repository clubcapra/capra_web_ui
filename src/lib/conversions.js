export let cToFahrenheit = c => c * 1.8 + 32
export let fToCelsius = f => (f - 32) / 1.8
export let toDegrees = radians => (radians * 180) / Math.PI
export let toAxis = quat => {
  const x = quat.x
  const y = quat.y
  const z = quat.z

  const r = 1 / Math.sqrt(x * x + y * y + z * z)

  return {
    x: x * r,
    y: y * r,
    z: z * r
  }
}
