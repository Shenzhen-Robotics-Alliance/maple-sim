# Maple Sim Docs Site

This folder contains the standalone Fumadocs site for Maple Sim. The original
markdown and assets remain in the repo root under `docs/`.

## Requirements

- Node.js (same version you use for the main repo)
- pnpm

## Install

```bash
pnpm install
```

`fumadocs-mdx` runs on install and generates the `.source/` directory used by
the app.

## Development

```bash
pnpm dev
```

Visit `http://localhost:3000`.

## Build (GitHub Pages)

The site is exported as static files. Use the base path for the Pages URL:

```bash
BASE_PATH=/maple-sim pnpm build
```

The static output is in `docs-site/out/`.

## Content & Assets

- Docs content lives in `docs-site/content/docs`.
- Static assets are copied from `docs/` into `docs-site/public` on dev/build:
  - `docs/media` → `docs-site/public/media`
  - `docs/javadocs` → `docs-site/public/javadocs`
  - `docs/vendordep` → `docs-site/public/vendordep`

## Troubleshooting

If you see errors like `fumadocs-mdx:collections` or
`useEffectEvent is not a function`, reinstall to refresh dependencies and
regenerate `.source`:

```bash
rm -rf node_modules .source .next
pnpm install
pnpm dev
```
