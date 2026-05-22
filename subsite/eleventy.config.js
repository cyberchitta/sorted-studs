import path from 'path';
import { fileURLToPath } from 'url';
import ejsPlugin from '@11ty/eleventy-plugin-ejs';
import sgPlugin from '@cyberchitta/supramental-gold/eleventy';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const sgRoot = path.dirname(
  fileURLToPath(import.meta.resolve('@cyberchitta/supramental-gold/package.json')),
);
const sgEleventy = path.join(sgRoot, 'eleventy');

export default function (eleventyConfig) {
  // Consumer `_includes` first so local files shadow SG defaults
  // (matches the main-site wiring).
  eleventyConfig.addPlugin(ejsPlugin, {
    views: [path.join(__dirname, '_includes'), sgEleventy],
  });
  eleventyConfig.addPlugin(sgPlugin);

  eleventyConfig.addGlobalData('layout', 'layouts/base.ejs');

  eleventyConfig.addPassthroughCopy('assets');

  eleventyConfig.ignores.add('README.md');
  eleventyConfig.ignores.add('CLAUDE.md');
  eleventyConfig.ignores.add('diverter_exhibit.md');

  return {
    dir: { input: '.', output: '_site', includes: '_includes', data: '_data' },
    markdownTemplateEngine: 'ejs',
    htmlTemplateEngine: 'ejs',
    templateFormats: ['md', 'ejs'],
  };
}
