// Minimal consumer helpers. SG's `base-chrome` data contract requires
// `helpers.stripHtml` (invoked only when a featured image carries a
// credit). Day-one sorted-studs has no featured images, so this is
// ready-for-use rather than load-bearing.

export default {
  stripHtml: (html) => (html ? html.replace(/<[^>]*>/g, '') : ''),
};
