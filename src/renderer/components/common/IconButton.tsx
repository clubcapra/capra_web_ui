import { styled } from '@/renderer/globalStyles/styled';
import React, { ReactNode } from 'react';

interface IconButtonProps {
  icon: ReactNode;
  onClick: () => void;
  title?: string;
  style?: React.CSSProperties;
}

const IconContainer = styled.div`
  display: flex;
  flex-direction: row;
  align-items: center;
  justify-content: center;
`;

const Button = styled.button`
  background-color: ${({ theme }) => theme.colors.darkerBackground};
  border-radius: 4px;
  padding: 6px;
  color: white;
  border: none;
  background: linear-gradient(#0000, rgb(0 0 0/30%)) top/100% 800%;
  transition: background-position 0.2s ease-in-out;

  &:hover {
    background-position: bottom;
  }
`;

const IconButton = ({ icon, onClick, title, style }: IconButtonProps) => {
  return (
    <Button style={style} onClick={onClick}>
      <IconContainer>
        {icon}
        {title && <span style={{ marginLeft: '4px' }}>{title}</span>}
      </IconContainer>
    </Button>
  );
};

export default IconButton;
