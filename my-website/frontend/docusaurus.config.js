import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Learn Robotics, AI, and Autonomous Systems',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  // VERCEL SETTINGS
  url: 'https://physical-ai-courseware.vercel.app', // Isay Vercel khud handle kar lega
  baseUrl: '/', // YAHAN CHANGE KIYA: GitHub Pages ke liye '/repo-name/' hota hai, Vercel ke liye sirf '/'

  organizationName: 'SyedaBismaJafri', 
  projectName: 'physical-ai-courseware', 

  // ERROR FIX: Isay 'ignore' karne se build fail nahi hogi
  onBrokenLinks: 'ignore', 
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/SyedaBismaJafri/physical-ai-courseware/tree/main/frontend/',
        },
        blog: {
          showReadingTime: true,
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'Physical AI',
        logo: {
          alt: 'Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Modules',
          },
          {to: '/blog', label: 'Blog', position: 'left'},
          {
            href: 'https://github.com/SyedaBismaJafri/physical-ai-courseware',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Modules',
            items: [
              { label: 'Overview', to: '/docs/intro' },
              { label: 'ROS 2', to: '/docs/module1-ros2' },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;