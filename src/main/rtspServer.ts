import { log } from '@/main/logger';
import { app, ipcMain } from 'electron';
import { ExecaChildProcess, execa } from 'execa';
import path from 'path';
import { isDev } from '@/main/isDev';

interface RtspProcess {
  process: ExecaChildProcess;
  wsPort: number;
}

const rtspServers: Map<string, RtspProcess> = new Map();

// Stack array of ports from (9000 to 9060) to use for rtsp servers
const ports = Array.from({ length: 61 }, (_, i) => i + 9000);

ipcMain.handle('rtsp_start', (event, url: string) => {
  const nextPort = ports.shift() ?? 9000;
  const process = execa('node', [
    isDev
      ? './script/rtspServer.js'
      : path.join(app.getAppPath(), '../renderer/script/rtspServer.js'),
    url,
    nextPort.toString(),
  ]);

  rtspServers.set(url, {
    process,
    wsPort: nextPort,
  });

  return nextPort;
});

ipcMain.on('rtsp_stop', (event, url: string) => {
  const rtspProcess = rtspServers.get(url);

  if (!rtspProcess) {
    return;
  }
  log.info('stopping rtsp process');
  ports.push(rtspProcess.wsPort);
  rtspProcess.process.kill();
  rtspServers.delete(url);
});
