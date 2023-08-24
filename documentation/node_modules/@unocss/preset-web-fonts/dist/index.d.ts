import { Preset } from '@unocss/core';

type WebFontsProviders = 'google' | 'bunny' | 'fontshare' | 'none' | Provider;
interface WebFontMeta {
    name: string;
    weights?: (string | number)[];
    italic?: boolean;
    /**
     * Override the provider
     * @default <matches root config>
     */
    provider?: WebFontsProviders;
}
interface ResolvedWebFontMeta extends Omit<WebFontMeta, 'provider'> {
    provider: Provider;
}
interface WebFontsOptions {
    /**
     * Provider service of the web fonts
     * @default 'google'
     */
    provider?: WebFontsProviders;
    /**
     * The fonts
     */
    fonts?: Record<string, WebFontMeta | string | (WebFontMeta | string)[]>;
    /**
     * Extend fonts to the theme object
     * @default true
     */
    extendTheme?: boolean;
    /**
     * Key for the theme object
     *
     * @default 'fontFamily'
     */
    themeKey?: string;
    /**
     * Inline CSS @import()
     *
     * @default true
     */
    inlineImports?: boolean;
    /**
     * Custom fetch function
     *
     * @default undefined
     */
    customFetch?: (url: string) => Promise<any>;
}
interface Provider {
    name: WebFontsProviders;
    getPreflight?(fonts: WebFontMeta[]): string;
    getImportUrl?(fonts: WebFontMeta[]): string | undefined;
    getFontName?(font: WebFontMeta): string;
}

declare function createGoogleCompatibleProvider(name: WebFontsProviders, host: string): Provider;

declare function normalizedFontMeta(meta: WebFontMeta | string, defaultProvider: WebFontsProviders): ResolvedWebFontMeta;
declare function preset(options?: WebFontsOptions): Preset<any>;

export { Provider, ResolvedWebFontMeta, WebFontMeta, WebFontsOptions, WebFontsProviders, createGoogleCompatibleProvider as createGoogleProvider, preset as default, normalizedFontMeta };
