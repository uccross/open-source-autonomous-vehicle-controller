# simple-git-hooks

A tool that lets you easily manage git hooks

- Zero dependency
- Small configuration (1 object in package.json)
- Lightweight:

## Usage

### Add simple-git-hooks to the project

1. Install simple-git-hooks as a dev dependency:

   ```sh
   npm install simple-git-hooks --save-dev
   ```

2. Add `simple-git-hooks` to your `package.json`. Fill it with git hooks and the corresponding commands.

   For example:

   ```jsonc
   {
     "simple-git-hooks": {
       "pre-commit": "npx lint-staged",
       "pre-push": "cd ../../ && npm run format",

       // All unused hooks will be removed automatically by default
       // but you can use the `preserveUnused` option like following to prevent this behavior

       // if you'd prefer preserve all unused hooks
       "preserveUnused": true,

       // if you'd prefer preserve specific unused hooks
       "preserveUnused": ["commit-msg"]
     }
   }
   ```

   This configuration is going to run all linters on every `commit` and formatter on `push`.

   > There are more ways to configure the package. Check out [Additional configuration options](#additional-configuration-options).

3. Run the CLI script to update the git hooks with the commands from the config:

   ```sh
   # [Optional] These 2 steps can be skipped for non-husky users
   git config core.hooksPath .git/hooks/
   rm -rf .git/hooks
   
   # Update ./git/hooks
   npx simple-git-hooks
   ```

Now all the git hooks are created.

### Update git hooks command

1. Change the configuration.

2. Run `npx simple-git-hooks` **from the root of your project**.

Note for **yarn2** users: Please run `yarn dlx simple-git-hooks` instead of the command above. More info on [dlx](https://yarnpkg.com/cli/dlx)

Note that you should manually run `npx simple-git-hooks` **every time you change a command**.

### Additional configuration options

You can also add a `.simple-git-hooks.cjs`, `.simple-git-hooks.js`, `simple-git-hooks.cjs`, `simple-git-hooks.js`, `.simple-git-hooks.json` or `simple-git-hooks.json` file to the project and write the configuration inside it.

This way `simple-git-hooks` configuration in `package.json` will not take effect any more.

`.simple-git-hooks.cjs`, `.simple-git-hooks.js` or `simple-git-hooks.cjs`, `simple-git-hooks.js` should look like the following.

```js
module.exports = {
  "pre-commit": "npx lint-staged",
  "pre-push": "cd ../../ && npm run format",
};
```

`.simple-git-hooks.json` or `simple-git-hooks.json` should look like the following.

```json
{
  "pre-commit": "npx lint-staged",
  "pre-push": "cd ../../ && npm run format"
}
```

If you need to have multiple configuration files or just your-own configuration file, you install hooks manually from it by `npx simple-git-hooks ./my-config.js`.

### Uninstall simple-git-hooks

> Uninstallation will remove all the existing git hooks.

```sh
npm uninstall simple-git-hooks
```

## Common issues

### When migrating from `husky` git hooks are not running

**Why is this happening?**

Husky might change the `core.gitHooks` value to `.husky`, this way, git hooks would search `.husky` directory instead of `.git/hooks/`.

Read more on git configuration in [Git book](https://git-scm.com/docs/githooks)

You can check it by running this command inside of your repo:

`git config core.hooksPath`

If it outputs `.husky` then this is your case

**How to fix?**

you need to point `core.gitHooks` value to `your-awesome-project/.git/hooks`. You can use this command:

`git config core.hooksPath .git/hooks/`

validate the value is set:

`git config core.hooksPath`

should output: `.git/hooks/`

Then remove the `.husky` folder that are generated previously by `husky`.
