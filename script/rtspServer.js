const Stream = require('node-rtsp-stream');

// Get node args
const args = process.argv.slice(2);

new Stream({
  name: 'RTSP-Stream',
  streamUrl: args[0],
  wsPort: parseInt(args[1]),
  ffmpegOptions: {
    // options ffmpeg flags
    '-stats': '', // an option with no neccessary value uses a blank string
    '-r': 30, // options with required values specify the value after the key
  },
});
