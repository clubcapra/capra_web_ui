import { styled } from '@/renderer/globalStyles/styled';

export const SectionTitle = styled.h2`
  padding-top: 12px;
  padding-bottom: 6px;
  margin-bottom: 6px;
  text-transform: uppercase;
  letter-spacing: 0.4pt;
  border-bottom: 1px solid ${({ theme }) => theme.colors.darkerBackground};
  font-size: inherit;
`;
