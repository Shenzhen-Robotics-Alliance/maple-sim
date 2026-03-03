import type { BaseLayoutProps } from 'fumadocs-ui/layouts/shared';
import { GithubInfo } from 'fumadocs-ui/components/github-info';
import Image from 'next/image';

const DiscordIcon = () => (
  <svg
    className="size-4"
    viewBox="0 0 24 24"
    role="img"
    aria-hidden="true"
  >
    <path
      fill="currentColor"
      d="M20.317 4.369a19.791 19.791 0 0 0-4.885-1.515.074.074 0 0 0-.079.037c-.211.375-.444.864-.608 1.249-1.844-.276-3.68-.276-5.486 0-.164-.394-.409-.874-.617-1.249a.077.077 0 0 0-.079-.037 19.736 19.736 0 0 0-4.885 1.515.07.07 0 0 0-.032.027C.533 9.045-.32 13.58.099 18.057a.082.082 0 0 0 .031.056 19.9 19.9 0 0 0 5.993 3.04.077.077 0 0 0 .084-.028c.462-.632.874-1.3 1.226-1.994a.076.076 0 0 0-.041-.106 13.107 13.107 0 0 1-1.872-.892.077.077 0 0 1-.007-.128c.126-.095.252-.192.371-.291a.074.074 0 0 1 .077-.01c3.927 1.793 8.18 1.793 12.061 0a.073.073 0 0 1 .078.01c.12.099.246.196.372.291a.077.077 0 0 1-.006.128 12.314 12.314 0 0 1-1.873.892.076.076 0 0 0-.04.106c.36.694.772 1.362 1.225 1.994a.076.076 0 0 0 .084.028 19.9 19.9 0 0 0 6.002-3.04.078.078 0 0 0 .03-.056c.5-5.177-.838-9.673-3.549-13.661a.061.061 0 0 0-.031-.028ZM8.02 15.331c-1.183 0-2.157-1.086-2.157-2.419 0-1.333.955-2.419 2.157-2.419 1.21 0 2.176 1.095 2.157 2.419 0 1.333-.955 2.419-2.157 2.419Zm7.974 0c-1.183 0-2.157-1.086-2.157-2.419 0-1.333.955-2.419 2.157-2.419 1.21 0 2.176 1.095 2.157 2.419 0 1.333-.946 2.419-2.157 2.419Z"
    />
  </svg>
);

export const baseOptions: BaseLayoutProps = {
  nav: {
    title: (
      <>
        <Image
          className="size-9 object-contain"
          src={`${process.env.BASE_PATH ?? ''}/media/team_logo.png`}
          alt="MapleSim Logo"
          width={36}
          height={36}
        />
        <span>MapleSim</span>
      </>
    ),
    url: '/'
  },
  links: [
    {
      type: 'custom',
      children: (
        <GithubInfo
          owner="Shenzhen-Robotics-Alliance"
          repo="maple-sim"
          className="lg:-mx-2"
        />
      )
    },
    {
      text: 'Discord',
      url: 'https://discord.gg/UsV8Qpwn',
      icon: <DiscordIcon />,
      type: 'icon'
    }
  ]
};
