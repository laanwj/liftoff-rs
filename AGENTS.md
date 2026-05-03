## General

- If anything is unclear, just ask. Don't assume things in the face of uncertainty.

## Code formatting

- No spaces at EOL.
- Newline at EOF.

## Code comments

- Don't ever add comments that document changes. Comments should *only* document the current state of the code. Do not refer to previous states of the code, even implicity. Comments must make sense for someone first reading the code.

## Git

- Make sure to only commit the files that belong to the change.

## Commit messages

- Do not add any special headers such as co-author to commit messages.

- The problem statement that describes the status quo is written in the present tense.  Write "The code does X when it is given input Y", instead of "The code used to do Y when given input X". You do not have to say "Currently"---the status quo in the problem statement is about the code _without_ your change, by project convention.

- Describe your changes in imperative mood, e.g. "make xyzzy do frotz" instead of "[This patch] makes xyzzy do frotz" or "[I] changed xyzzy to do frotz", as if you are giving orders to the codebase to change its behavior. Try to make sure your explanation can be understood without external resources.


