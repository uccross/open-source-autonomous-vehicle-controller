import { L as LoadConfigOptions, a as LoadConfigResult } from './types-6b9af6ab.js';
export { B as BuiltinParsers, C as CustomParser, b as LoadConfigSource, S as SearchOptions, d as defaultExtensions } from './types-6b9af6ab.js';
import '@antfu/utils';

declare function createConfigLoader<T>(options: LoadConfigOptions): {
    load: (force?: boolean) => Promise<LoadConfigResult<T>>;
    findConfigs: () => Promise<string[]>;
};
declare function loadConfig<T>(options: LoadConfigOptions): Promise<LoadConfigResult<T>>;

export { LoadConfigOptions, LoadConfigResult, createConfigLoader, loadConfig };
