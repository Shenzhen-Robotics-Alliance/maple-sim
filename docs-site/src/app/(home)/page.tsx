import type { Metadata } from 'next';
import Link from 'next/link';
// import { HeroSearchButton } from '@/components/hero-search';

export const metadata: Metadata = {
  title: 'MapleSim'
};

export default function HomePage() {
  const basePath = process.env.BASE_PATH ?? '';

  return (
    <main className="relative min-h-screen overflow-hidden bg-white px-4 py-20 text-slate-900 dark:bg-[#0f1014] dark:text-white">
      <div className="pointer-events-none absolute inset-0">
        <div className="absolute -top-40 left-1/2 h-[520px] w-[520px] -translate-x-1/2 rounded-full bg-red-200/70 blur-[140px] dark:bg-[#ef4444]/30" />
        <div className="absolute bottom-0 right-0 h-[420px] w-[420px] translate-x-1/3 rounded-full bg-pink-200/60 blur-[140px] dark:bg-[#ec4899]/25" />
      </div>

      <div className="relative mx-auto grid w-full max-w-6xl gap-12 lg:grid-cols-[1.15fr_0.85fr] lg:items-center">
        <div>
          <div className="flex flex-wrap items-center gap-3">
            <p className="text-sm font-semibold uppercase tracking-[0.2em] text-slate-500 dark:text-white/60">
              Physics-first robot simulation
            </p>
            <span className="rounded-full border border-slate-200 bg-white/80 px-3 py-1 text-xs font-semibold uppercase tracking-[0.2em] text-slate-500 dark:border-white/15 dark:bg-white/10 dark:text-white/70">
              Beta Docs
            </span>
          </div>
          <h1 className="mt-4 text-5xl font-bold leading-tight sm:text-6xl">
            MapleSim
          </h1>
          <p className="mt-5 text-lg text-slate-700 dark:text-white/80">
            Elevating FRC Java Robot Simulations to the Next Level with Physics
            Engines.
          </p>
          <p className="mt-4 text-base text-slate-600 dark:text-white/70">
            A simulation engine is a powerful tool that provides realistic
            approximations of physical systems. With MapleSim, we integrate the
            open-source Java rigid-body dynamics engine, dyn4j, capable of
            simulating 2D forces and collisions between rigid shapes.
          </p>

          <div className="mt-8 flex flex-col gap-4 sm:flex-row">
            <Link
              href="/docs/installing-maple-sim"
              className="rounded-full bg-[#ef4444] px-8 py-3 text-center font-semibold text-white transition hover:bg-[#dc2626]"
            >
              Get Started
            </Link>
            <Link
              href="/docs/simulation-details"
              className="rounded-full border border-slate-200 bg-white px-8 py-3 text-center font-semibold text-slate-900 transition hover:bg-slate-100 dark:border-white/20 dark:bg-white/10 dark:text-white dark:hover:bg-white/20"
            >
              Explore Docs
            </Link>
            {/* <HeroSearchButton /> */}

          </div>

          {/* <div className="mt-6 rounded-2xl border border-white/10 bg-white/5 p-4 text-sm text-white/75">
            You are reading the documentation for a Beta version of MapleSim. API
            references are subject to change in future versions.
          </div> */}

          <div className="mt-10 grid gap-6 sm:grid-cols-3">
            {[
              [
                'Realistic interactions',
                'Engage directly with robots, field elements, and game pieces.'
              ],
              [
                'Test autonomous modes',
                'Evaluate autonomous routines with pinpoint accuracy.'
              ],
              [
                'Fine-tune subsystems',
                'Optimize shooters and other mechanisms with simulated physics.'
              ]
            ].map(([title, desc]) => (
              <div
                key={title}
                className="rounded-2xl border border-slate-200 bg-white/80 p-4 text-sm text-slate-700 dark:border-white/10 dark:bg-white/5 dark:text-white/80"
              >
                <h3 className="text-base font-semibold text-slate-900 dark:text-white">{title}</h3>
                <p className="mt-2">{desc}</p>
              </div>
            ))}
          </div>
        </div>

        <div className="rounded-3xl border border-slate-200 bg-white/80 p-6 dark:border-white/10 dark:bg-white/5">
          <div className="flex items-center justify-center rounded-2xl border border-slate-200 bg-slate-900/5 p-6 dark:border-white/10 dark:bg-black/40">
            <img
              src={`${basePath}/media/team_logo.png`}
              alt="MapleSim logo"
              className="h-40 w-40 object-contain sm:h-48 sm:w-48"
            />
          </div>
          <div className="relative mt-4 overflow-hidden rounded-2xl border border-slate-200 bg-slate-900/5 p-4 dark:border-white/10 dark:bg-black/40">
            <img
              src={`${basePath}/media/demo%20video%20cover.png`}
              alt="MapleSim demo preview"
              className="h-auto w-full rounded-xl object-cover"
            />
            <a
              href="https://www.youtube.com/watch?v=CBx1_Dosgec"
              target="_blank"
              rel="noreferrer"
              className="absolute inset-x-0 bottom-6 mx-auto w-fit rounded-full bg-white/90 px-5 py-2 text-sm font-semibold text-black shadow transition hover:bg-white"
            >
              Watch Demo Video
            </a>
          </div>
          <div className="mt-6 space-y-4 text-sm text-slate-600 dark:text-white/75">
            <div>
              <p className="font-semibold text-slate-900 dark:text-white">Built for teams</p>
              <p className="mt-1">
                Practice driving, validate auton paths, and iterate on control
                loops in a realistic environment.
              </p>
            </div>
            {/* <div className="rounded-xl border border-white/10 bg-white/5 p-3 text-xs text-white/75">
              MapleSim is integrated with AdvantageKit Swerve Templates and
              officially included in YAGSL for 2025.
            </div> */}
            <div className="flex flex-col gap-3 sm:flex-row">
              <a
                href="https://github.com/Shenzhen-Robotics-Alliance/maple-sim"
                target="_blank"
                rel="noreferrer"
                className="rounded-full border border-slate-200 px-6 py-2 text-center font-semibold text-slate-900 transition hover:bg-slate-100 dark:border-white/15 dark:text-white dark:hover:bg-white/10"
              >
                GitHub
              </a>
              <a
                href="https://discord.gg/UsV8Qpwn"
                target="_blank"
                rel="noreferrer"
                className="rounded-full border border-slate-200 px-6 py-2 text-center font-semibold text-slate-900 transition hover:bg-slate-100 dark:border-white/15 dark:text-white dark:hover:bg-white/10"
              >
                Discord
              </a>
            </div>
          </div>
        </div>
      </div>
    </main>
  );
}
