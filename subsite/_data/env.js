// Sibling-site URL helper. Netlify sets `CONTEXT` and `BRANCH` per build;
// in branch-deploy contexts we derive the sibling's preview URL from the
// same branch name + the sibling's Netlify slug. Production and local dev
// fall through to the sibling's canonical production URL.
const branch = process.env.BRANCH;
const isBranchDeploy = process.env.CONTEXT === 'branch-deploy' && branch;
const siblingUrl = (slug, prodUrl) =>
  isBranchDeploy ? `https://${branch}--${slug}.netlify.app` : prodUrl;

export default {
  eleventyEnv: process.env.ELEVENTY_ENV,
  siteUrl: process.env.DEPLOY_PRIME_URL || 'https://sorted-studs.cyberchitta.cc',
  mainSiteUrl: siblingUrl('cyberchitta', 'https://www.cyberchitta.cc'),
};
