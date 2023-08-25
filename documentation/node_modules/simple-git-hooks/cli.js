#!/usr/bin/env node

const {setHooksFromConfig} = require('./simple-git-hooks')

try {
    setHooksFromConfig(process.cwd(), process.argv)
    console.log('[INFO] Successfully set all git hooks')
} catch (e) {
    console.log('[ERROR], Was not able to set git hooks. Error: ' + e)
}
