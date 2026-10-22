// Stop and cancel queued movement/settings before asking the firmware to reset.
// An ACK acknowledges transport only; REINICIO_TOTAL_OK confirms actual erasure.
export async function resetRobot(send: (command: string) => Promise<boolean>, cancel: () => void) {
  cancel();
  if (!(await send('x'))) return false;
  return send('resetall');
}
