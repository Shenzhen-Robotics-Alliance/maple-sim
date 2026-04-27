import type { ImgHTMLAttributes } from 'react';
import type { MDXComponents } from 'mdx/types';
import defaultMdxComponents from 'fumadocs-ui/mdx';
import * as Tabs from 'fumadocs-ui/components/tabs';
import * as Accordions from 'fumadocs-ui/components/accordion';
import { withBasePath } from '@/lib/base-path';

type ImgSrc = ImgHTMLAttributes<HTMLImageElement>['src'] | { src: string };

function resolveImgSrc(src: ImgSrc) {
  if (typeof src === 'string') return src;
  if (src && typeof src === 'object' && 'src' in src) return src.src;
  return undefined;
}

function Img({ src, ...props }: ImgHTMLAttributes<HTMLImageElement>) {
  const rawSrc = resolveImgSrc(src as ImgSrc);
  const resolvedSrc =
    typeof rawSrc === 'string' && rawSrc.startsWith('/')
      ? withBasePath(rawSrc)
      : rawSrc;
  return <img src={resolvedSrc} {...props} />;
}

export function getMDXComponents(components: MDXComponents = {}): MDXComponents {
  return {
    ...defaultMdxComponents,
    ...Tabs,
    ...Accordions,
    img: Img,
    ...components
  };
}
