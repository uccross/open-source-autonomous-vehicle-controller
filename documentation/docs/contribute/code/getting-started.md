# Getting Started

## Sign into GitHub

Sign into your GitHub account, or [create a free GitHub account](https://github.com/join?ref=dataschool.io) if you don't have one.

## Fork the project repository

Go to the [OSAVC project's repository](https://github.com/uccross/open-source-autonomous-vehicle-controller) on GitHub, and then "fork" it by clicking the Fork button in the upper right corner:
![Fork OSAVC Repo](../../assets/images/contribute/code/Fork.png)

This creates a copy of the project repository in your GitHub account. In the upper left corner, you will see that you are now looking at a repository in your account:

![Your Fork](../../assets/images/contribute/code/YourFork.png)

## Clone your fork

While still in your repository, click the green Clone or download button and then copy the HTTPS URL:

![Clone your fork](../../assets/images/contribute/code/Clone.png)

Using Git on your local machine, clone your fork using the URL you just copied:

``` bash
git clone URL_OF_FORK.
```

For example, I used

```bash
git clone https://github.com/Aniruddha1261/open-source-autonomous-vehicle-controller.git
```

Cloning copies the repository files (and commit history) from GitHub to your local machine. The repository will be downloaded into a subdirectory of your working directory, and the subdirectory will have the same name as the repository.

(If you run into problems during this step, read the [Set up Git](https://docs.github.com/en/get-started/quickstart/set-up-git?ref=dataschool.io) page from GitHub's documentation.)

## Navigate to your local repository

Since the clone was downloaded into a subdirectory of your working directory, you can navigate to it using:

```bash
cd NAME_OF_REPOSITORY.
```

For example, I used

```bash
cd open-source-autonomous-vehicle-controller
```

## Check that your fork is the "origin" remote

You are going to be synchronizing your local repository with both the project repository (on GitHub) and your fork (also on GitHub). The URLs that point to these repositories are called "remotes". More specifically, the project repository is called the "upstream" remote, and your fork is called the "origin" remote.

When you cloned your fork, that should have automatically set your fork as the "origin" remote. Use **git remote -v** to show your current remotes. You should see the URL of your fork (which you copied in step 3) next to the word "origin".

If you don't see an "origin" remote, you can add it using:

```bash
git remote add origin URL_OF_FORK.
```

(If you run into problems during this step, read the [Managing remote repositories](https://docs.github.com/en/get-started/getting-started-with-git/managing-remote-repositories?ref=dataschool.io) page from GitHub's documentation.)

## Create a new branch

Rather than making changes to the project's "master" branch, it's a good practice to instead create your own branch. This creates an environment for your work that is isolated from the master branch.

Use

```bash
git checkout -b BRANCH_NAME
```

to create a new branch and then immediately switch to it. The name of the branch should briefly describe what you are working on, and should not contain any spaces.

For example, I used

```bash
git checkout -b doc
```

because I was making some small fixes to the documentation.

Use git branch to show your local branches. You should see your new branch as well as "master", and your new branch should have an asterisk next to it to indicate that it's "checked out" (meaning that you're working in it).
