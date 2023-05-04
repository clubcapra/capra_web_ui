declare module 'node-rtsp-stream' {
  class Stream {
    constructor(params: {
      name: string;
      streamUrl: string;
      wsPort: number;
      ffmpegOptions: {
        '-stats': string;
        '-r': number;
      };
    });
  }
}
