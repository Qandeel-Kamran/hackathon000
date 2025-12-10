// @ts-check
// `@ts-check` enables ts-autocomments on the config

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Comprehensive Guide to Embodied Intelligence and Humanoid Systems',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-site.com', // TODO: Update this to your site's URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-organization', // Usually your GitHub org/user name.
  projectName: 'physical-ai-documentation', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
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
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-username/physical-ai-documentation/tree/main/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  plugins: [
    // Recommended plugin for search functionality
    [
      '@docusaurus/theme-search-algolia',
      {
        // The application ID provided by Algolia
        appId: process.env.ALGOLIA_APP_ID || 'YOUR_APP_ID',
        // Public API key: it is safe to commit it
        apiKey: process.env.ALGOLIA_API_KEY || 'YOUR_SEARCH_API_KEY',
        indexName: process.env.ALGOLIA_INDEX_NAME || 'your_index_name',
        // Optional: see doc section below
        contextualSearch: true,
        // Optional: Specify domains where the navigation should occur through window.location instead of history.push. Useful when our Algolia config crawls multiple documentation sites and we want to navigate with window.location.href to them.
        externalUrlRegex: 'external\\.com|domain\\.com',
        // Optional: Replace parts of the item URLs from Algolia. Useful when using the same search index for multiple deployments using a different baseUrl. You can use regexp or string in the `from` param. For example: 1. The search interface deployed at https://example.com bases all links on the absolute path /docs/; 2. The same search index contains entry with path /docs/; 3. The application may wish to change these paths to be based on the baseUrl, resulting in /baseUrl/docs/. Another example: 1. The search interface deployed at https://example.com bases all links on the absolute path /docs/; 2. The same search index contains entry with path /docs/; 3. The application may wish to remove the /docs/ prefix, resulting in links based on the root path instead of /docs/.
        replaceSearchResultPathname: {
          from: '/docs/', // or as RegExp: /\/docs\//
          to: '/',
        },
        // Optional: Algolia search parameters
        searchParameters: {},
        // Optional: path for search page that enabled by default (`false` to disable it)
        searchPagePath: 'search',
      },
    ],
    // Plugin for mathematical notation support
    [
      'docusaurus-plugin-math',
      {
        before_mdx: [],
        after_mdx: [],
        remark: {
          math: true,
          katex: true,
        },
        rehype: {
          katex: true,
        },
      },
    ],
    // Plugin for image zoom functionality
    [
      'docusaurus-plugin-image-zoom',
      {
        selector: '.markdown img',
        config: {
          background: 'rgba(0,0,0,0.8)',
          scrollOffset: 50,
          zIndex: 9999,
        },
      },
    ],
    // Plugin for Mermaid diagram support
    [
      '@docusaurus/theme-mermaid',
      {
        mermaid: {
          theme: 'default',
        },
      },
    ],
    // Plugin for client-side redirects
    [
      '@docusaurus/plugin-client-redirects',
      {
        redirects: [
          // Example redirect configuration
          // {
          //   to: '/docs/new-location',
          //   from: ['/docs/old-location', '/docs/legacy-location'],
          // },
        ],
      },
    ],
    // Plugin for privacy-focused analytics with Matomo
    [
      'docusaurus-plugin-matomo',
      {
        siteId: process.env.MATOMO_SITE_ID || '1',
        url: process.env.MATOMO_URL || 'https://your-matomo-instance.com',
      },
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'docsSidebar',
            position: 'left',
            label: 'Documentation',
          },
          {
            href: 'https://github.com/your-username/physical-ai-documentation',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Introduction',
                to: '/docs/category/introduction',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/physical-ai',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/physical-ai',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-username/physical-ai-documentation',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['python', 'cpp', 'json', 'bash'],
      },
    }),
};

module.exports = config;