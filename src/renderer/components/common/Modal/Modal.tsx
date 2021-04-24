import * as React from 'react'
import {
  StyledModal,
  StyledOverlay,
  StyledModalClose,
  StyledModalTitle,
  StyledModalContent,
  StyledModalHeader,
} from '@/renderer/components/common/Modal/Modal.styles'
import { FaTimes } from 'react-icons/fa'
import { ReactNode } from 'react'

interface Props {
  isOpen: boolean
  onClose: () => void
  title: string
  footer?: ReactNode
}

export const Modal: React.FC<Props> = ({
  isOpen,
  onClose,
  title,
  children,
  footer,
}) => {
  return isOpen ? (
    <>
      <StyledOverlay />
      <StyledModal>
        <StyledModalHeader>
          <StyledModalTitle>{title}</StyledModalTitle>
          <StyledModalClose onClick={onClose}>
            <FaTimes />
          </StyledModalClose>
        </StyledModalHeader>
        <StyledModalContent>{children}</StyledModalContent>
        {footer}
      </StyledModal>
    </>
  ) : null
}
