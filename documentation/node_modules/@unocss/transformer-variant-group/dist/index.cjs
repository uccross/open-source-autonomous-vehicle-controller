'use strict';

const core = require('@unocss/core');

function transformerVariantGroup(options = {}) {
  return {
    name: "@unocss/transformer-variant-group",
    enforce: "pre",
    transform(s) {
      core.expandVariantGroup(s, options.separators);
    }
  };
}

module.exports = transformerVariantGroup;
