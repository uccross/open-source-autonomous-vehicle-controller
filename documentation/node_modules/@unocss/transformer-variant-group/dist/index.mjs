import { expandVariantGroup } from '@unocss/core';

function transformerVariantGroup(options = {}) {
  return {
    name: "@unocss/transformer-variant-group",
    enforce: "pre",
    transform(s) {
      expandVariantGroup(s, options.separators);
    }
  };
}

export { transformerVariantGroup as default };
