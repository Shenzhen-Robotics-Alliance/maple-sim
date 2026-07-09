import type { Config } from 'tailwindcss';

const config: Config = {
  content: [
    './src/**/*.{ts,tsx,mdx}',
    './content/**/*.{md,mdx}',
    './src/mdx-components.tsx'
  ],
  theme: {
    extend: {}
  },
  plugins: []
};

export default config;
