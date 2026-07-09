import './global.css';
import type { ReactNode } from 'react';
import { Provider } from '@/app/provider';
import { Inter } from 'next/font/google';

const inter = Inter({
  subsets: ['latin']
});

const basePath = process.env.BASE_PATH ?? '';

export const metadata = {
  icons: {
    icon: [
      { url: `${basePath}/favicon.ico` },
      { url: `${basePath}/favicon.png`, sizes: '512x512', type: 'image/png' }
    ]
  }
};

export default function Layout({ children }: { children: ReactNode }) {
  return (
    <html lang="en" className={inter.className} suppressHydrationWarning>
      <body className="flex min-h-screen flex-col">
        <Provider>{children}</Provider>
      </body>
    </html>
  );
}
