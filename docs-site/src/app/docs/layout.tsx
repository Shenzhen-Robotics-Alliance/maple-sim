import type { ReactNode } from 'react';
import { DocsLayout } from 'fumadocs-ui/layouts/notebook';
import { baseOptions } from '@/app/layout.config';
import { source } from '@/lib/source';
import Link from 'next/link';

function QuickstartRail() {
  return (
    <div className="rounded-xl border border-fd-border bg-fd-background p-3 text-xs">
      <p className="font-semibold text-fd-foreground">Quickstart</p>
      <div className="mt-2 flex flex-col gap-2 text-fd-muted-foreground">
        <Link className="hover:text-fd-foreground" href="/docs/installing-maple-sim">
          Install MapleSim
        </Link>
        <Link className="hover:text-fd-foreground" href="/docs/using-the-simulated-arena">
          Simulated Arena
        </Link>
        <Link className="hover:text-fd-foreground" href="/docs/swerve-simulation-overview">
          Swerve Simulation
        </Link>
        <Link className="hover:text-fd-foreground" href="/docs/simulating-intake">
          Simulating Intake
        </Link>
        <Link className="hover:text-fd-foreground" href="/docs/simulating-projectiles">
          Simulating Projectiles
        </Link>
      </div>
    </div>
  );
}

export default function Layout({ children }: { children: ReactNode }) {
  return (
    <DocsLayout
      {...baseOptions}
      tabMode="navbar"
      nav={{ ...baseOptions.nav, mode: 'top' }}
      sidebar={{ banner: <QuickstartRail /> }}
      tree={source.pageTree}
    >
      {children}
    </DocsLayout>
  );
}
