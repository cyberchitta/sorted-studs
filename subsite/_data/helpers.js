// Minimal consumer helpers. SG's `base-chrome` data contract requires
// `helpers.stripHtml` (invoked only when a featured image carries a
// credit). `article-body` additionally requires `getAuthorImage` when
// a `showrunner` is set, and `renderMarkdown` when `updates` carry
// notes or a `cta` is set.

import { readFileSync } from 'fs';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';
import { marked } from 'marked';
import env from './env.js';

const subsiteRoot = join(dirname(fileURLToPath(import.meta.url)), '..');

const AUTHOR_IMAGES = {
  '@restlessronin': `${env.mainSiteUrl}/assets/images/authors/restlessronin.webp`,
};

// Inline an SVG so page CSS reaches its content. cad-khana writes
// .cad-visible / .cad-hidden classes on themeable polylines (run
// `khana draw --format svg --themeable`); the consumer's CSS binds
// those classes to design-system color tokens.
//
// Trimmed because the result lands in markdown: a trailing newline plus
// the template line's own makes a blank line, which ends markdown's HTML
// block and turns the next SVG into paragraph text — `<br>` between its
// elements breaks the parser out of SVG and blanks the rest of the page.
// The XML prolog is meaningless inline and goes too.
const inlineSvg = (relPath) => {
  const abs = join(subsiteRoot, relPath.replace(/^\//, ''));
  return readFileSync(abs, 'utf8')
    .replace(/^\s*<\?xml[^>]*\?>\s*/, '')
    .trim();
};

export default {
  stripHtml: (html) => (html ? html.replace(/<[^>]*>/g, '') : ''),
  getAuthorImage: (tag) =>
    AUTHOR_IMAGES[tag] ||
    `${env.mainSiteUrl}/assets/images/authors/unknown.webp`,
  renderMarkdown: (md) => (md ? marked.parse(md) : ''),
  inlineSvg,
};
