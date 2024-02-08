# Contributing

Before making changes to the system, ensure that it is up-to-date by running

```sh
$(cd src/arena/arena-rosnav && git pull)
vcs import src < src/arena/arena-rosnav/.repos
rosdep update && rosdep install --from-paths src --ignore-src -r -y
cd src/arena/arena-rosnav && poetry install && cd ../../..
```

## Contributing to Arena-Rosnav/arena-rosnav

Modifying the `arena-rosnav` repository is trivial.

Commit your changes to the `arena-rosnav` repository. Push the changes to your fork and pull request the branch to `Arena-Rosnav:master`.

## Contributing to Arena-Rosnav/*

When modifying packages that are part of the `Arena-Rosnav` organization, but _not_ part of the main `arena-rosnav` repository; commit, push, and pull request the changes to the respective repositories.

Once the pull requests are merged, update the commit hash in `src/arena/arena-rosnav/.repos` and pull request the change.

## Dependencies on External Tools

If you need external packages that _aren't_ installable using `rosdep`, add the respective repository to `src/extern/`.

Add the repository to `src/arena/arena-rosnav/.repos`. A copyable configuration can be generated using

```sh
vcs export --exact --repos src/extern/<name_of_the_new_repo>
```