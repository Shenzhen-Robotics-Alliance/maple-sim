const isProd = process.env.NODE_ENV === 'production';

export const BASE_PATH =
  process.env.BASE_PATH ?? (isProd ? '/maple-sim' : '');

export function withBasePath(path: string): string {
  if (!BASE_PATH) return path;
  if (!path.startsWith('/')) return path;
  return `${BASE_PATH}${path}`;
}
