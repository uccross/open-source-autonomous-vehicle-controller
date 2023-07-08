# Review the pull request

You have now created a pull request, which is stored in the project's repository (not in your fork of the repository). It's a good idea to read through what you wrote, as well as clicking on the **Commits** tab and the **Files changed** tab to review the contents of your pull request

If you realize that you left out some important details, you can click the 3 dots in the upper right corner to edit your pull request description.

## Add more commits to your pull request

You can continue to add more commits to your pull request even after opening it! For example, the project maintainers may ask you to make some changes, or you may just think of a change that you forgot to include:

Start by returning to your local repository, and use **git branch** to see which branch is currently checked out. If you are currently in the master branch (rather than the branch you created), then use **git checkout BRANCH_NAME** to switch. For example, I used git checkout doc.

Then, you should repeat given above:

- make changes, commit them, and push them to your fork.

Finally, return to your open pull request on GitHub and refresh the page. You will see that your new commits have automatically been added to the pull request:

## Discuss the pull request

If there are questions or discussion about your pull request from the project maintainers, you can add to the conversation using the comment box at the bottom of the pull request:

If there are inline comments about specific changes you made, you can respond to those as well:

Click the Resolve conversation button once you have addressed any specific requests.

## Delete your branch from your fork

If the project maintainers accept your pull request (congratulations!), they will merge your proposed changes into the project's master branch and close your pull request:

You will be given the option to delete your branch from your fork, since it's no longer of any use:

## Delete your branch from your local repository

You should also delete the branch you created from your local repository, so that you don't accidentally start working in it the next time you want to make a contribution to this project.

First, switch to the master branch: git checkout master.

Then, delete the branch you created: git branch -D BRANCH_NAME. For example, I used git branch -D doc

## Synchronize your fork with the project repository

At this point, your fork is out of sync with the project repository's master branch.

To get it back in sync, you should first use Git to pull the latest changes from "upstream" (the project repository) into your local repository: **git pull upstream master**.

Then, push those changes from your local repository to the "origin" (your fork): **git push origin master**.

If you return to your fork on GitHub, you will see that the master branch is "even" with the project repository's master branch
