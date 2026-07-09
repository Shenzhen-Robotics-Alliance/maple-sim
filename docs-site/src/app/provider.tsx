'use client';

import type { ReactNode } from 'react';
import { RootProvider } from 'fumadocs-ui/provider';

export function Provider({ children }: { children: ReactNode }) {
  return <RootProvider>{children}</RootProvider>;
}
