import fs from 'node:fs';
import path from 'node:path';

const root = path.resolve(process.cwd());
const docsRoot = path.resolve(root, '..', 'docs');
const publicRoot = path.join(root, 'public');

const mappings = [
  { src: path.join(docsRoot, 'media'), dest: path.join(publicRoot, 'media') },
  { src: path.join(docsRoot, 'javadocs'), dest: path.join(publicRoot, 'javadocs') },
  { src: path.join(docsRoot, 'vendordep'), dest: path.join(publicRoot, 'vendordep') }
];

if (!fs.existsSync(publicRoot)) {
  fs.mkdirSync(publicRoot, { recursive: true });
}

for (const { src, dest } of mappings) {
  if (!fs.existsSync(src)) {
    console.warn(`[sync-public-assets] Missing source: ${src}`);
    continue;
  }

  fs.rmSync(dest, { recursive: true, force: true });
  fs.cpSync(src, dest, { recursive: true });
}
