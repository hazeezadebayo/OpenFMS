# Git Workflow and Discipline

This document defines the explicit version control actions and rules for this repository. All version control actions must be intentional, documented, and justified.

## Initial Setup
- `git init`: Initialize a new Git repository.
- `git remote add origin <url>`: Link the local repository to a remote Github repository.
- `git push -u origin main`: Push the initial commit and set upstream tracking.

## Common Workflows

### Clone
- `git clone <url>`: Clone an existing remote repository to local. Use when setting up the project on a new machine.

### Branching
- `git branch <branch-name>`: Create a new branch. Use for developing new features or bug fixes in isolation.
- `git checkout -b <branch-name>`: Create and switch to a new branch in one command.
- `git checkout <branch-name>`: Switch to an existing branch.

### Committing
- `git add <file>`: Stage specific changes for commit. Never use `git add .` unless absolutely sure of every changed file.
- `git commit -m "<message>"`: Commit staged changes with a descriptive message explaining *why* the change was made, not just *what* changed.

### Syncing
- `git push`: Push committed changes to the remote branch. Use frequently to backup work.
- `git pull`: Fetch and merge changes from the remote branch to local. Use before starting work or pushing to avoid conflicts.
- `git fetch`: Download objects and refs from another repository without merging them. Use to inspect remote changes before merging.

### Integrating
- `git merge <branch>`: Merge the specified branch into the current one. Use to integrate completed feature branches into main.
- `git rebase <branch>`: Reapply commits on top of another base tip. Use to keep a linear history, but **never rebase commits that exist outside your local repository**.

## Recovery Commands (Handle with Care)

> [!WARNING]
> The following commands rewrite history or discard changes. Use extreme caution.

- `git restore <file>`: Discard local uncommitted changes to a file. **Risk**: Cannot be undone.
- `git reset --soft HEAD~1`: Undo the last commit but keep changes staged. Use when you need to amend the previous commit message or add more changes.
- `git reset --hard HEAD`: Discard all uncommitted changes. **Risk**: Complete loss of uncommitted work.
- `git revert <commit>`: Create a new commit that undoes the changes from a previous commit. **Safe** for public branches.
- `git stash`: Temporarily shelve changes to work on something else. Use `git stash pop` to re-apply.
- `git reflog`: View the reference logs to find lost commits. Use as a last resort to recover from destructive commands.

## Important Rules
- Never force push (`git push -f`) to public branches (like `main`).
- Always run `git status` and `git diff` before adding and committing to ensure only intended changes are included.
- `.gitignore` must be strictly maintained (e.g., ignoring `__pycache__`, environment files, IDE configs).
