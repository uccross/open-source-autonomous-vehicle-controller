# @jcamp/tailwindcss-plugin-icons

<div align="center">
  <img src="./.github/tailwindcss-mark.svg" alt="Tailwind CSS" width="108" height="66">
  <h1>Tailwind CSS Icons Plugin</h1>
  <p>Automatic icon creator for Tailwind CSS</p>
</div>

Use **any** icons with **Pure CSS** for [TailwindCSS](https://tailwindcss.com).

Recommended reading - [@antfu](https://github.com/antfu)'s post titled [Icons in Pure CSS](https://antfu.me/posts/icons-in-pure-css).

Follow the following conventions to use the icons

- `<prefix>[<collection>-<icon>]`
- `i-[<collection>-<icon>]` - default prefix is `i-`

For examples:

```html
<!-- A basic anchor icon from Phosphor icons -->
<div class="i-[ph-anchor-simple-thin]" />
<!-- An orange alarm from Material Design Icons -->
<div class="i-[mdi-alarm] text-orange-400" />
<!-- A large Vue logo -->
<div class="i-[logos-vue] text-3xl" />
<!-- Sun in light mode, Moon in dark mode, from Carbon -->
<button class="i-[carbon-sun] dark:i-[carbon-moon]" />
<!-- Twemoji of laugh, turns to tear on hovering -->
<div class="i-[twemoji-grinning-face-with-smiling-eyes] hover:i-[twemoji-face-with-tears-of-joy]" />
```

<img src="https://user-images.githubusercontent.com/11247099/136709053-31b4db79-eddc-4dc6-aa2d-388086332630.gif" height="100"><br><sup>This is powered by pure CSS</sup>

## Install

```bash
npm i -D @jcamp/tailwindcss-plugin-icons @iconify-json/[the-collection-you-want]
```

We use [Iconify](https://iconify.design) as our data source of icons. You need to install the corresponding iconset in `devDependencies` by following the `@iconify-json/*` pattern. For example, `@iconify-json/mdi` for [Material Design Icons](https://materialdesignicons.com/), `@iconify-json/tabler` for [Tabler](https://tabler-icons.io/). You can refer to [Icônes](https://icones.js.org/) or [Iconify](https://icon-sets.iconify.design/) for all the collections available.

Then add the plugin to your `tailwind.config.js` file:

```js
// tailwind.config.js
const icons = require('@jcamp/tailwindcss-plugin-icons')
module.exports = {
  theme: {
    // ...
  },
  plugins: [
    icons({
      /* options */
    }),
    // ...
  ],
}
```

If you prefer to install the all the icon sets available on Iconify at once (~130MB):

```bash
npm i -D @iconify/json
```

## Class names

The structure is `i-[<collection>/<name>/<scaling>]`

The collection and name can be divided by either a dash `-` or a slash `/`. Unfortunately, Tailwind CSS will not pass a colon `:` properly to the plugin.

If you want to add scaling to the class, you must separate that with a slash `/`. You can add the unit `(px|em|rem)` but it is optional. If omitted, it will use the default specified by the config.

```html
<div class="i-[logos-vue/2]" />
<div class="i-[logos/vue/24px]" />
<div class="i-[logos-vue/2rem]" />
```

## JSON collections

As a big fan of Font Awesome, I wanted this to work with their pro collections. It now does. There is a new `jsonCollections` property in the config that allows you to tell the plugin where to load JSON files from.

```js
jsonCollections: {
  custom: 'json/custom-collection.json',
},
```

## Configuration

Refer to the [type definition](https://github.com/jcamp-code/tailwindcss-plugin-icons/blob/main/src/types.ts) for all configurations avaliable.

### Extra Properties

You can provide the extra CSS properties to control the default behavior of the icons. The following is an example of make icons inlined by default:

```ts
presetIcons({
  extraProperties: {
    display: 'inline-block',
    'vertical-align': 'middle',
    // ...
  },
})
```

## Modes Overriding

By default, this preset will choose the rendering modes automatically for each icon based on the icons' characteristics. You can read more in this [blog post](https://antfu.me/posts/icons-in-pure-css). In some cases, you may want to explicitly set the rendering modes for each icon.

- `i-bg-` for `background-img` - renders the icon as a background image
- `i-mask-` for `mask` - renders the icon as a mask image

### Browser

Because Tailwind does not allow async plugins, see https://github.com/tailwindlabs/tailwindcss/discussions/7277, we are using a node process to retrieve the icons, so right now, this will not work in the browser directly.

#### Customization

Although there is code similar to UnoCSS's customizations, I have not yet had time to write tests for it and confirm it works as expected, so this information will not be in the README for now.

### Node.js

In `Node.js` the preset will search for the installed iconify dataset automatically and so you don't need to register the `iconify` collections.

## Credits

This plugin is heaviliy inspired by and based on the work of [UnoCSS Icons Preset](https://github.com/unocss/unocss/tree/main/packages/preset-icons) created by [Anthony Fu](https://github.com/antfu)

It is also based on some ideas by [InfiniteXyy](https://github.com/InfiniteXyy) and their work at [tailwindcss-iconify](https://github.com/InfiniteXyy/tailwindcss-iconify). In particular, their idea for spawning a node process to allow async code to work in Tailwinds sync plugin system.

## License

MIT License &copy; 2022-PRESENT [John Campion](https://github.com/JohnCampionJr/)
