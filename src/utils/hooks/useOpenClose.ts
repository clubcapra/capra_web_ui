import { useState } from 'react'

export const useOpenClose = (
  defaultIsOpen: boolean = false
): [boolean, () => void, () => void, () => void] => {
  const [isOpen, setIsOpen] = useState(defaultIsOpen)
  const onClose = () => setIsOpen(false)
  const onOpen = () => setIsOpen(true)
  const onToggle = () => setIsOpen(!isOpen)
  return [isOpen, onOpen, onClose, onToggle]
}
