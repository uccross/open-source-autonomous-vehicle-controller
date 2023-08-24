'use strict';

const AstroIntegrationPlugin = require('@unocss/astro');
const presetUno = require('@unocss/preset-uno');

function _interopDefaultLegacy (e) { return e && typeof e === 'object' && 'default' in e ? e["default"] : e; }

const AstroIntegrationPlugin__default = /*#__PURE__*/_interopDefaultLegacy(AstroIntegrationPlugin);
const presetUno__default = /*#__PURE__*/_interopDefaultLegacy(presetUno);

function UnocssAstroIntegration(config) {
  return AstroIntegrationPlugin__default(
    config,
    {
      presets: [
        presetUno__default()
      ]
    }
  );
}

module.exports = UnocssAstroIntegration;
