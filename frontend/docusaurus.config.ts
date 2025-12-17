import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'Interactive Learning for AI-Native Robotics Education',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://robotics-textbook.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'robotics-textbook', // Usually your GitHub org/user name.
  projectName: 'ai-native-robotics', // Usually your repo name.

  onBrokenLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Inject backend URL for client-side use
  customFields: {
    BACKEND_URL: 'http://localhost:8000',
  },

    presets: [
    [
      'classic',
      {
        docs: {
          // Serve docs from frontend/docs/book so Docusaurus ignores placeholder files
          path: 'docs/book',
          routeBasePath: 'docs',
          sidebarPath: require.resolve('./sidebars.js'),
          include: ['**/*.md', '**/*.mdx'],
        },

        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Textbook',
      logo: {
        alt: 'Physical AI Robotics Logo',
        src: 'img/logo.svg',
      },
        items: [],
    },
    footer: {
      style: 'dark',
      copyright: `Physical AI & Humanoid Robotics Textbook<br/>
      A comprehensive guide to AI-native robotics education<br/>
      Copyright © ${new Date().getFullYear()} Robotics Education Project`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'cpp'],
    },
  } satisfies Preset.ThemeConfig,

  // Inject backend URL script
  scripts: [
    `window.__BACKEND_URL__ = 'http://localhost:8000';`,
  ],
};

export default config;
