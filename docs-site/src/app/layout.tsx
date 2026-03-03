import './global.css';
import type { ReactNode } from 'react';
import { Provider } from '@/app/provider';

export default function Layout({ children }: { children: ReactNode }) {
  return (
    <html lang="en" suppressHydrationWarning>
      <body>
        <Provider>{children}</Provider>
      </body>
    </html>
  );
}
