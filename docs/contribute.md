# Contributing

Before making changes to the system, ensure that it is up-to-date by running

```sh
rosrun arena_bringup pull
```

### Adding Python Packages

Add python packages to the environment by running 
```sh
cd src/arena/arena-rosnav
poetry add my_package
```

## Contributing to Arena-Rosnav/arena-rosnav

Modifying the `arena-rosnav` repository is trivial.

Commit your changes to the `arena-rosnav` repository.

## Dependencies on External Tools

If you need external packages that _aren't_ installable using `rosdep`, add the respective repository to `src/extern/`.

Add the repository to `src/arena/arena-rosnav/.repos/<package>.repos` (use `rosrun arena_bringup repos` as an interactive tool). Finally, pull request the `.repos` change and provide a short description.
