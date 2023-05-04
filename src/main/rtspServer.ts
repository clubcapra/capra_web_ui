import { log } from '@/main/logger';
import { ipcMain } from 'electron';
import { execaNode, ExecaChildProcess } from 'execa';

interface RtspProcess {
  process: ExecaChildProcess;
  wsPort: number;
}

const rtspServers: Map<string, RtspProcess> = new Map();
let nextPort = 9000;

ipcMain.handle('rtsp_start', (event, url: string) => {
  const process = execaNode('script.ts');

  log.info('starting rtsp process');
  rtspServers.set(url, {
    process,
    wsPort: nextPort,
  });

  return nextPort++;
});

ipcMain.on('rtsp_stop', (event, url: string) => {
  const rtspProcess = rtspServers.get(url);

  if (!rtspProcess) {
    return;
  }

  rtspProcess.process.kill();
  rtspServers.delete(url);
});
