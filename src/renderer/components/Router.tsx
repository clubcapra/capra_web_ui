import React, { FC } from 'react';
import { Route, Routes, Navigate } from 'react-router-dom';
import { Teleop } from '@/renderer/components/pages/Teleop';
import { Victim } from '@/renderer/components/pages/Victim';
import { Debug } from '@/renderer/components/pages/Debug';
import { ConfigPage } from '@/renderer/components/pages/Config/ConfigPage';

export const Router: FC = () => {
  return (
    <>
      <Routes>
        <Route path="/markhor" element={<Navigate to="/markhor/teleop" />} />
        <Route path="/markhor/teleop" element={<Teleop />} />
        <Route path="/markhor/victim" element={<Victim />} />
        <Route path="/markhor/config/*" element={<ConfigPage />} />
        <Route path="/markhor/debug" element={<Debug />} />
        <Route path="/" element={<Navigate to="/markhor/teleop" />} />
        <Route path="/rove" element={<Navigate to="/rove/teleop" />} />
        <Route path="/rove/teleop" element={<Teleop />} />
        <Route path="/rove/victim" element={<Victim />} />
        <Route path="/rove/config/*" element={<ConfigPage />} />
        <Route path="/rove/debug" element={<Debug />} />
      </Routes>
    </>
  );
};
