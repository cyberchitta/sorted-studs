import { resolveSiteUrl } from '@cyberchitta/supramental-gold/eleventy/site-url';

// Sibling-site URL helper. A non-production deploy must never link to a
// production deploy — a click-through from a draft would fire the sibling's
// analytics — so branch deploys link to the sibling's own preview deploy.
// The sibling's preview branch is declared here explicitly, not derived from
// this site's `BRANCH`, so the two repos' branch layouts are free to differ.
// The sibling declares ours the same way in its own env data; change both
// when either preview branch moves. Production and local dev fall through
// to the sibling's canonical production URL.
const isBranchDeploy = process.env.CONTEXT === 'branch-deploy';
const siblingUrl = (slug, previewBranch, prodUrl) =>
  isBranchDeploy ? `https://${previewBranch}--${slug}.netlify.app` : prodUrl;

export default {
  eleventyEnv: process.env.ELEVENTY_ENV,
  siteUrl: resolveSiteUrl('https://sorted-studs.cyberchitta.cc'),
  mainSiteUrl: siblingUrl('cyberchitta', 'staging', 'https://www.cyberchitta.cc'),
};
