import { dirname, join } from 'path';
import { fileURLToPath } from 'url';
import { readingTimeForFile } from '@cyberchitta/supramental-gold/eleventy/build-helpers';
import env from './env.js';

const __dirname = dirname(fileURLToPath(import.meta.url));
const articleRoot = join(__dirname, '..');

const featuredImage = {
  url: `${env.mainSiteUrl}/assets/images/heroes/lego-scanner-tight-26050702.webp`,
  ogFormat: 'png',
  credit:
    "Rotating Turret Scanner vibe Caded with @claude-opus-4.7 using the <a href='https://github.com/cyberchitta/cad-khana'>cad-khana</a> skill.",
};

export default {
  title: 'sorted-studs',
  tagline: 'AI-powered LEGO sorting machine',
  description:
    'An AI-powered LEGO sorting machine. Mechanical singulation, four-camera multi-view scanning, ML classification, automated bin output. Building in public.',
  url: env.siteUrl,
  mainSiteUrl: env.mainSiteUrl,
  github: 'cyberchitta/sorted-studs',
  featuredImages: {
    '/': featuredImage,
  },
  images: [
    {
      url: featuredImage.url,
      width: 907,
      height: 942,
    },
  ],
  articles: [{ url: '/', readingTime: readingTimeForFile(join(articleRoot, 'index.md')) }],
};
