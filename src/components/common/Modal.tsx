import * as React from 'react'
import {
  StyledModal,
  StyledOverlay,
  StyledModalClose,
} from 'components/common/Modal.styles'
import { FaTimes } from 'react-icons/fa'

interface Props {
  isOpen: boolean
  onClose: () => void
}

export const Modal: React.FC<Props> = ({ isOpen, onClose, children }) => {
  return isOpen ? (
    <>
      <StyledOverlay />
      <StyledModalClose onClick={onClose}>
        <FaTimes />
      </StyledModalClose>
      <StyledModal>{children}</StyledModal>
    </>
  ) : null
}
