import type { Metadata } from 'next';
import Link from 'next/link';

export const metadata: Metadata = {
  title: 'MapleSim'
};

export default function HomePage() {
  const basePath = process.env.BASE_PATH ?? '';

  return (
    <main className="relative min-h-screen overflow-hidden bg-[#0f1014] px-4 py-20 text-white">
      <div className="pointer-events-none absolute inset-0">
        <div className="absolute -top-40 left-1/2 h-[520px] w-[520px] -translate-x-1/2 rounded-full bg-[#ef4444]/30 blur-[140px]" />
        <div className="absolute bottom-0 right-0 h-[420px] w-[420px] translate-x-1/3 rounded-full bg-[#ec4899]/25 blur-[140px]" />
      </div>

      <div className="relative mx-auto grid w-full max-w-6xl gap-12 lg:grid-cols-[1.15fr_0.85fr] lg:items-center">
        <div>
          <p className="text-sm font-semibold uppercase tracking-[0.2em] text-white/60">
            Physics-first robot simulation
          </p>
          <h1 className="mt-4 text-5xl font-bold leading-tight sm:text-6xl">
            MapleSim
          </h1>
          <p className="mt-5 text-lg text-white/80">
            Elevating FRC Java Robot Simulations to the Next Level with Physics
            Engines.
          </p>
          <p className="mt-4 text-base text-white/70">
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
              className="rounded-full border border-white/20 bg-white/10 px-8 py-3 text-center font-semibold text-white transition hover:bg-white/20"
            >
              Explore Docs
            </Link>
          </div>

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
                className="rounded-2xl border border-white/10 bg-white/5 p-4 text-sm text-white/80"
              >
                <h3 className="text-base font-semibold text-white">{title}</h3>
                <p className="mt-2">{desc}</p>
              </div>
            ))}
          </div>
        </div>

        <div className="rounded-3xl border border-white/10 bg-white/5 p-6">
          <div className="flex items-center justify-center rounded-2xl border border-white/10 bg-black/40 p-6">
            <img
              src={`${basePath}/media/team_logo.png`}
              alt="MapleSim logo"
              className="h-40 w-40 object-contain sm:h-48 sm:w-48"
            />
          </div>
          <div className="rounded-2xl border border-white/10 bg-black/40 p-4">
            <img
              src={`${basePath}/media/demo%20video%20cover.png`}
              alt="MapleSim demo preview"
              className="h-auto w-full rounded-xl object-cover"
            />
          </div>
          <div className="mt-6 space-y-4 text-sm text-white/75">
            <div>
              <p className="font-semibold text-white">Built for teams</p>
              <p className="mt-1">
                Practice driving, validate auton paths, and iterate on control
                loops in a realistic environment.
              </p>
            </div>
            <div className="flex flex-col gap-3 sm:flex-row">
              <a
                href="https://github.com/Shenzhen-Robotics-Alliance/maple-sim"
                target="_blank"
                rel="noreferrer"
                className="rounded-full border border-white/15 px-6 py-2 text-center font-semibold text-white transition hover:bg-white/10"
              >
                GitHub
              </a>
              <a
                href="https://discord.gg/UsV8Qpwn"
                target="_blank"
                rel="noreferrer"
                className="rounded-full border border-white/15 px-6 py-2 text-center font-semibold text-white transition hover:bg-white/10"
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
