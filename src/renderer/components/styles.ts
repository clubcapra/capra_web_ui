import Popup from 'reactjs-popup';
import { styled } from '@/renderer/globalStyles/styled';

export const StyledPopup = styled(Popup)`
  @keyframes anvil {
    0% {
      transform: scale(1) translateY(0px);
      opacity: 0;
      box-shadow: 0 0 0 rgba(241, 241, 241, 0);
    }
    1% {
      transform: scale(0.96) translateY(10px);
      opacity: 0;
      box-shadow: 0 0 0 rgba(241, 241, 241, 0);
    }
    100% {
      transform: scale(1) translateY(0px);
      opacity: 1;
      box-shadow: 0 0 500px rgba(241, 241, 241, 0);
    }
  }
  &-content {
    -webkit-animation: anvil 0.2s cubic-bezier(0.38, 0.1, 0.36, 0.9) forwards;
  }
`;

export const StyledPopupContent = styled.div`
  display: flex;
  flex-direction: row;
  align-items: flex-start;
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  box-shadow: 0 0 10px 0 rgba(0, 0, 0, 0.5);
  border-radius: 4px;
  min-width: 150px;
`;

export const StyledPopupContainer = styled.div`
  display: flex;
  flex-direction: row;
  justify-content: center;
  align-items: center;
  &:hover {
    cursor: pointer;
  }
`;
