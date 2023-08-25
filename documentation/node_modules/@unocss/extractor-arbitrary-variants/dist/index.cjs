'use strict';

Object.defineProperty(exports, '__esModule', { value: true });

const core = require('@unocss/core');

const quotedArbitraryValuesRE = /(?:[\w&:[\]-]|\[\S+=\S+\])+\[\\?['"]?\S+?['"]\]\]?[\w:-]*/g;
const arbitraryPropertyRE = /\[(\\\W|[\w-])+:[^\s:]*?("\S+?"|'\S+?'|`\S+?`|[^\s:]+?)[^\s:]*?\)?\]/g;
const arbitraryPropertyCandidateRE = /^\[(\\\W|[\w-])+:['"]?\S+?['"]?\]$/;
function splitCodeWithArbitraryVariants(code) {
  const result = /* @__PURE__ */ new Set();
  for (const match of code.matchAll(arbitraryPropertyRE)) {
    if (!code[match.index - 1]?.match(/^[\s'"`]/))
      continue;
    result.add(match[0]);
  }
  for (const match of code.matchAll(quotedArbitraryValuesRE))
    result.add(match[0]);
  code.split(core.defaultSplitRE).forEach((match) => {
    if (core.isValidSelector(match) && !arbitraryPropertyCandidateRE.test(match))
      result.add(match);
  });
  return [...result];
}
const extractorArbitraryVariants = {
  name: "@unocss/extractor-arbitrary-variants",
  order: 0,
  extract({ code }) {
    return splitCodeWithArbitraryVariants(code);
  }
};

exports.arbitraryPropertyRE = arbitraryPropertyRE;
exports["default"] = extractorArbitraryVariants;
exports.extractorArbitraryVariants = extractorArbitraryVariants;
exports.quotedArbitraryValuesRE = quotedArbitraryValuesRE;
exports.splitCodeWithArbitraryVariants = splitCodeWithArbitraryVariants;
