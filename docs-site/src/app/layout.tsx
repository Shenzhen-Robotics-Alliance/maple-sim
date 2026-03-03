import './global.css';
import type { ReactNode } from 'react';
import { Provider } from '@/app/provider';
import { Inter } from 'next/font/google';

const inter = Inter({
  subsets: ['latin']
});

export const metadata = {
  icons: {
    icon: `${process.env.BASE_PATH ?? ''}/favicon.ico`
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
