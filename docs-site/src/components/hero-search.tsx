'use client';

import { useSearchContext } from 'fumadocs-ui/contexts/search';

export function HeroSearchButton() {
  const { enabled, setOpenSearch } = useSearchContext();

  if (!enabled) return null;

  return (
    <button
      type="button"
      className="rounded-full border border-slate-200 bg-white px-6 py-3 text-center font-semibold text-slate-900 transition hover:bg-slate-100 dark:border-white/20 dark:bg-white/10 dark:text-white dark:hover:bg-white/20"
      onClick={() => setOpenSearch(true)}
    >
      <span className="inline-flex items-center gap-2">
        <svg className="size-4" viewBox="0 0 24 24" aria-hidden="true">
          <path
            fill="currentColor"
            d="M21.53 20.47l-4.35-4.35a8 8 0 1 0-1.06 1.06l4.35 4.35a.75.75 0 1 0 1.06-1.06ZM5.5 10a4.5 4.5 0 1 1 9 0a4.5 4.5 0 0 1-9 0Z"
          />
        </svg>
        Search Docs
      </span>
    </button>
  );
}
