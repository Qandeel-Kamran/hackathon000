# Physical AI & Humanoid Robotics Documentation

This repository contains the complete documentation for the book "Physical AI & Humanoid Robotics" published as a Docusaurus site.

## Installation

To install and run the documentation site locally:

```bash
# Clone the repository
git clone <repository-url>
cd physical-ai-documentation

# Install dependencies
npm install

# Or if using yarn
yarn install
```

## Development

To start a local development server with hot reloading:

```bash
npm run start
# or
yarn start
```

This will start the development server at `http://localhost:3000` with live reload functionality.

## Build

To build the static site for production:

```bash
npm run build
# or
yarn build
```

The build command creates an optimized static site in the `build/` directory that can be deployed to any static hosting service.

## Recommended Plugins

The documentation site uses several recommended plugins for enhanced functionality:

- `@docusaurus/plugin-content-docs` - For documentation site functionality
- `@docusaurus/plugin-content-blog` - For blog functionality (if needed)
- `@docusaurus/plugin-google-gtag` - For Google Analytics
- `@docusaurus/plugin-sitemap` - For automatic sitemap generation
- `@docusaurus/theme-search-algolia` - For search functionality
- `@docusaurus/plugin-client-redirects` - For URL redirects
- `docusaurus-plugin-matomo` - For privacy-focused analytics
- `@docusaurus/plugin-content-pages` - For custom pages

Additional useful plugins:
- `docusaurus-plugin-image-zoom` - For image zooming functionality
- `@docusaurus/theme-mermaid` - For Mermaid diagram rendering
- `docusaurus-plugin-math` - For mathematical notation support

## Deploy to Vercel

To deploy your Docusaurus site to Vercel:

1. Push your code to a Git repository (GitHub, GitLab, or Bitbucket)
2. Go to [Vercel](https://vercel.com) and create a new project
3. Import your repository
4. Set the build command to `npm run build` (or `yarn build`)
5. Set the output directory to `build`
6. Add any required environment variables
7. Deploy!

Vercel will automatically rebuild and deploy your site on every push to the main branch.

## Environment Variables

The following environment variables can be configured:

- `ALGOLIA_API_KEY` - For Algolia search functionality
- `ALGOLIA_APP_ID` - For Algolia search functionality
- `ALGOLIA_INDEX_NAME` - For Algolia search functionality
- `MATOMO_URL` - For Matomo analytics
- `MATOMO_SITE_ID` - For Matomo analytics

## Contributing

To contribute to the documentation:

1. Fork the repository
2. Create a new branch for your changes
3. Make your changes to the Markdown files in the `docs/` directory
4. Test your changes locally with `npm run start`
5. Submit a pull request

## License

This documentation is licensed under [LICENSE TYPE] - see the LICENSE file for details.