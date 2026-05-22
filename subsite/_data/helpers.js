// Minimal consumer helpers. SG's `base-chrome` data contract requires
// `helpers.stripHtml` (invoked only when a featured image carries a
// credit). `article-body` additionally requires `getAuthorImage` when
// a `showrunner` is set, and `renderMarkdown` when `updates` carry
// notes or a `cta` is set.

import { readFileSync } from 'fs';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';
import { marked } from 'marked';

const subsiteRoot = join(dirname(fileURLToPath(import.meta.url)), '..');

const AUTHOR_IMAGES = {
  '@restlessronin':
    'https://www.cyberchitta.cc/assets/images/authors/restlessronin.webp',
  '@claude-opus-4.7':
    'https://www.cyberchitta.cc/assets/images/authors/claude.webp',
};

// Inline an SVG so page CSS reaches its content. cad-khana writes
// .cad-visible / .cad-hidden classes on themeable polylines (run
// `khana draw --format svg --themeable`); the consumer's CSS binds
// those classes to design-system color tokens.
const inlineSvg = (relPath) => {
  const abs = join(subsiteRoot, relPath.replace(/^\//, ''));
  return readFileSync(abs, 'utf8');
};

export default {
  stripHtml: (html) => (html ? html.replace(/<[^>]*>/g, '') : ''),
  getAuthorImage: (tag) =>
    AUTHOR_IMAGES[tag] ||
    'https://www.cyberchitta.cc/assets/images/authors/unknown.webp',
  renderMarkdown: (md) => (md ? marked.parse(md) : ''),
  inlineSvg,
};
