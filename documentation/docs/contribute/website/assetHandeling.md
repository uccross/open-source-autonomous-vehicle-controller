# Asset Handling

## Referencing Static Assets

All Markdown files are compiled into Vue components and processed by [Vite](https://vitejs.dev/guide/assets.html). You can, **and should**, reference any assets using relative URLs:

```md
![An image](./image.png)
```

You can reference static assets in your markdown files, your `*.vue` components in the theme, styles and plain `.css` files either using absolute public paths (based on project root) or relative paths (based on your file system). The latter is similar to the behavior you are used to if you have used Vite, Vue CLI, or webpack's `file-loader`.

Common image, media, and font filetypes are detected and included as assets automatically.

All referenced assets, including those using absolute paths, will be copied to the output directory with a hashed file name in the production build. Never-referenced assets will not be copied. Image assets smaller than 4kb will be base64 inlined - this can be configured via the `vite` config option.

All **static** path references, including absolute paths, should be based on your working directory structure.
