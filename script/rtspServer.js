const Stream = require('node-rtsp-stream');

// Get node args
const args = process.argv.slice(2);

// Check if required arguments are present
if (args.length < 2) {
  const filename = __filename.split('/').pop();
  // eslint-disable-next-line no-console
  console.error(
    `Missing Arguments! Usage: node ${filename} <rtsp-url> <websocket-port>`
  );
  process.exit(1);
}

new Stream({
  name: 'RTSP-Stream',
  streamUrl: args[0],
  wsPort: parseInt(args[1]),
  ffmpegOptions: {
    // options ffmpeg flags
    '-re': '', // read input at native frame rate
    '-q': 0, // quality video in scale [0, 32]
  },
});
