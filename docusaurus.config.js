// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics text-book',
  tagline: 'Master Physical AI and Humanoid Robotics with Interactive Tutorials',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://aurangzaibmughal.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: '/Ai--book-/',

  // GitHub pages deployment config.
  organizationName: 'aurangzaibmughal',
  projectName: 'Ai--book-',

  onBrokenLinks: 'throw',

  // Internationalization
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
    localeConfigs: {
      en: {
        label: 'English',
        direction: 'ltr',
        htmlLang: 'en-US',
      },
      ur: {
        label: 'اردو',
        direction: 'rtl',
        htmlLang: 'ur-PK',
      },
    },
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/aurangzaibmughal/Ai--book-/edit/main/',
        },
        blog: false,
        theme: {
          customCss: [
            require.resolve('./src/css/custom.css'),
            require.resolve('./src/css/rtl.css'),
          ],
        },
      }),
    ],
  ],

  themes: ['@docusaurus/theme-mermaid'],
  markdown: {
    mermaid: true,
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  plugins: [
    [
  require.resolve("@easyops-cn/docusaurus-search-local"),
  {
    hashed: true,
    language: ["en"],
    highlightSearchTermsOnTargetPage: true,
    docsRouteBasePath: "/docs",
    indexDocs: true,
    indexBlog: false,
    indexPages: false,
    // Note: Urdu content will be indexed but without language-specific stemming
    // as lunr-languages doesn't support Urdu yet
  },
 ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // SEO metadata
      metadata: [
        {name: 'keywords', content: 'Physical AI, Humanoid Robotics, robotics, embodied AI, robot learning, manipulation, locomotion, tutorial, textbook, education, Urdu'},
        {name: 'description', content: 'Interactive Physical AI and Humanoid Robotics textbook with hands-on labs, exercises, and multilingual support. Learn robotics and embodied AI with comprehensive tutorials in English and Urdu.'},
        {name: 'author', content: 'Physical AI & Humanoid Robotics text-book'},
        {property: 'og:type', content: 'website'},
        {property: 'og:title', content: 'Physical AI & Humanoid Robotics text-book - Learn Robotics Interactively'},
        {property: 'og:description', content: 'Interactive robotics textbook with hands-on labs, exercises, quizzes, and multilingual support'},
        {property: 'og:image', content: 'img/logo.svg'},
        {name: 'twitter:card', content: 'summary_large_image'},
        {name: 'twitter:title', content: 'Physical AI & Humanoid Robotics text-book'},
        {name: 'twitter:description', content: 'Learn Physical AI and Humanoid Robotics with interactive tutorials and hands-on labs'},
      ],
      image: 'img/logo.svg',
      navbar: {
        title: 'Physical AI & Robotics',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Chapters',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/aurangzaibmughal/Ai--book-',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Learn',
            items: [
              {
                label: 'Chapters',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/aurangzaibmughal/Ai--book-',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics text-book. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'javascript', 'typescript', 'bash'],
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
    }),
};

module.exports = config;
