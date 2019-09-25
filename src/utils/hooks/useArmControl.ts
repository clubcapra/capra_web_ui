import { useDispatch } from 'react-redux'
import { useEffect } from 'react'
import { gamepadSlice } from 'store/modules/gamepad/reducer'

export const useArmControl = (isArmControlled = true) => {
  const dispatch = useDispatch()
  useEffect(() => {
    dispatch(gamepadSlice.actions.setIsArmControlled(isArmControlled))
    return () => {
      dispatch(gamepadSlice.actions.setIsArmControlled(!isArmControlled))
    }
  }, [isArmControlled, dispatch])
}
