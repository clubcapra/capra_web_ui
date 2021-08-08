import * as React from 'react'
import { FaTimes } from 'react-icons/fa'
import { ReactNode } from 'react'
import { styled } from '@/renderer/globalStyles/styled'

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

const StyledModal = styled.div`
  height: auto;
  width: auto;
  min-width: 500px;
  position: fixed;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  z-index: 999;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.32), 0 2px 6px rgba(0, 0, 0, 0.21);
  background-color: ${({ theme }) => theme.colors.background};
`

const StyledModalHeader = styled.div`
  display: flex;
  border-bottom: 1px solid ${({ theme }) => theme.colors.darkerBackground};
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.32), 0 2px 6px rgba(0, 0, 0, 0.21);
`

const StyledModalTitle = styled.h3`
  padding: 5px;
  flex-grow: 9;
`

const StyledModalClose = styled.div`
  flex-grow: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  cursor: pointer;

  &:hover {
    background-color: red;
  }

  svg {
    height: 24px;
    width: 24px;
  }
`

const StyledModalContent = styled.div`
  padding: 5px;
`

const StyledOverlay = styled.div`
  position: fixed;
  top: 0;
  left: 0;
  width: 100vw;
  height: 100vh;
  background-color: rgba(0, 0, 0, 0.4);
  z-index: 998;
`
