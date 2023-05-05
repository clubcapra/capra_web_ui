declare module '@cycjimmy/jsmpeg-player' {
  class VideoElement {
    constructor(
      videoWrapper: HTMLDivElement,
      url: string,
      options: {
        canvas?: HTMLCanvasElement;
        autoplay?: boolean;
        audio?: boolean;
      }
    );

    destroy(): void;
  }
}
