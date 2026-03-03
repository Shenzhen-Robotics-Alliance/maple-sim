import { createPreset } from 'fumadocs-ui/tailwind-plugin';
import type { Config } from 'tailwindcss';

const config: Config = {
  presets: [createPreset()],
  content: [
    './src/**/*.{ts,tsx,mdx}',
    './content/**/*.{md,mdx}',
    './src/mdx-components.tsx'
  ]
};

export default config;
