import { createMDX } from 'fumadocs-mdx/next';

if (
  typeof globalThis.localStorage !== 'undefined' &&
  typeof globalThis.localStorage?.getItem !== 'function'
) {
  try {
    delete globalThis.localStorage;
  } catch {
    globalThis.localStorage = undefined;
  }
}

const withMDX = createMDX();

const basePath = process.env.BASE_PATH ?? '';

/** @type {import('next').NextConfig} */
const config = {
  reactStrictMode: true,
  devIndicators: false,
  output: 'export',
  basePath,
  images: {
    unoptimized: true
  },
  env: {
    BASE_PATH: basePath
  }
};

export default withMDX(config);
