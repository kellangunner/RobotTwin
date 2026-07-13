// Type shim for the Emscripten-emitted ES6 module (build artifact, not
// checked in — produced by `emcmake cmake -S . -B build-wasm && cmake --build build-wasm`).
// eslint-disable-next-line @typescript-eslint/no-explicit-any
declare const createRobotTwinModule: (options?: Record<string, unknown>) => Promise<any>;
export default createRobotTwinModule;
