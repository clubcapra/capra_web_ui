import { log } from '@/main/logger';
import { app, ipcMain } from 'electron';
import { ExecaChildProcess, execa } from 'execa';
import path from 'path';
import { isDev } from '@/main/isDev';
import { RTSP_START, RTSP_STOP } from './preload';

interface RtspProcess {
  process: ExecaChildProcess;
  wsPort: number;
}

const rtspServers: Map<number, RtspProcess> = new Map();

// Stack array of ports from (9000 to 9060) to use for rtsp servers
const ports = Array.from({ length: 61 }, (_, i) => i + 9000);

ipcMain.handle(RTSP_START, (_, url: string) => {
  const nextPort = ports.shift() ?? 9000;
  const process = execa('node', [
    isDev
      ? './script/rtspServer.js'
      : path.join(app.getAppPath(), '../renderer/script/rtspServer.js'),
    url,
    nextPort.toString(),
  ]);

  rtspServers.set(nextPort, {
    process,
    wsPort: nextPort,
  });

  return nextPort;
});

ipcMain.on(RTSP_STOP, (_, port: number) => {
  const rtspProcess = rtspServers.get(port);

  if (!rtspProcess) {
    return;
  }
  log.info('stopping rtsp process');
  ports.push(rtspProcess.wsPort);
  rtspProcess.process.kill();
  rtspServers.delete(port);
});
