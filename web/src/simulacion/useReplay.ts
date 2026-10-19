import { useCallback, useEffect, useRef, useState } from 'react';
import { ReplaySession } from './ReplaySession';
import type { ReplayData } from '../telemetria/types';
export function useReplay() {
  const [session, setSession] = useState<ReplaySession | null>(null),
    [, update] = useState(0);
  const generation = useRef(0);
  const open = useCallback((data: ReplayData) => {
    generation.current++;
    setSession(new ReplaySession(data));
  }, []);
  useEffect(() => {
    if (!session) return;
    let last = performance.now();
    const timer = setInterval(() => {
      const now = performance.now();
      if (session.playing) {
        session.advance((now - last) / 1000);
        update((n) => n + 1);
      }
      last = now;
    }, 40);
    return () => clearInterval(timer);
  }, [session]);
  return {
    session,
    open,
    generation,
    close: () => {
      generation.current++;
      setSession(null);
    },
    seek: (t: number) => {
      if (session) {
        session.playing = false;
        session.seek(t);
        update((n) => n + 1);
      }
    },
    toggle: () => {
      if (session) {
        if (session.time >= session.duration) session.seek(0);
        session.playing = !session.playing;
        update((n) => n + 1);
      }
    },
    speed: (n: number) => {
      if (session) {
        session.speed = n;
        update((n) => n + 1);
      }
    },
  };
}
