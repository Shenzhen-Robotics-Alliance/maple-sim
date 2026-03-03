import type { ReactNode } from 'react';
import { DocsLayout } from 'fumadocs-ui/layouts/notebook';
import { baseOptions } from '@/app/layout.config';
import { source } from '@/lib/source';

export default function Layout({ children }: { children: ReactNode }) {
  return (
    <DocsLayout
      {...baseOptions}
      tabMode="navbar"
      nav={{ ...baseOptions.nav, mode: 'top' }}
      tree={source.pageTree}
    >
      {children}
    </DocsLayout>
  );
}
