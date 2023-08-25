# Make changes in your local repository

Use a text editor or IDE to make the changes you planned to the files in your local repository. Because you checked out a branch in the previous step, any edits you make will only affect that branch.

## Commit your changes

After you make a set of changes, use

```bash
git add -A
```

to stage your changes and

```bash
git commit -m "DESCRIPTION OF CHANGES"
```

to commit them.

For example, I used

```bash
git commit -m "fix typos in Documentation"
```

for one of my commits.

If you are making multiple sets of changes, it's a good practice to make a commit after each set.

## Push your changes to your fork

When you are done making all of your changes, upload these changes to your fork using

```bash
git push origin BRANCH_NAME
```

This "pushes" your changes to the "BRANCH_NAME" branch of the "origin" (which is your fork on GitHub).

For example, I used

```bash
git push origin doc-fixes
```

## Begin the pull request

Return to your fork on GitHub, and refresh the page. You may see a highlighted area that displays your recently pushed branch:

![Begin Pull Request](../../assets/images/contribute/code/CompareChanges.png)

Click the Compare & pull request button to begin the pull request.

(Alternatively, if you don't see this highlighted area, you can switch to your branch using the Branch button and then click the New pull request button.)

## Create the pull request

When opening a "pull request", you are making a "request" that the project repository "pull" changes from your fork. You will see that the project repository is listed as the "base repository", and your fork is listed as the "head repository":

![Pull Request](../../assets/images/contribute/code/PullRequest.png)

Before submitting the pull request, you first need to describe the changes you made (rather than asking the project maintainers to figure them out on their own). You should write a descriptive title for your pull request, and then include more details in the body of the pull request. If there are any related GitHub issues, make sure to mention those by number. The body can include Markdown formatting, and you can click the **Preview** tab to see how it will look.

Below the pull request form, you will see a list of the commits you made in your branch, as well as the "diffs" for all of the files you changed.

If everything looks good, click the green **Create pull request** button!
