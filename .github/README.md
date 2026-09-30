# Repository automation

The workflows target the `ros2` branch.

| Workflow | Purpose |
| --- | --- |
| `autoware-index-validate` | Build, run package tests, and run clang-tidy on Humble and Jazzy for pull requests and pushes to `ros2`, or on manual dispatch. |
| `pre-commit` | Check repository files and workflow syntax without requiring the pre-commit.ci service. |
| `semantic-pull-request` | Validate conventional commit style PR titles. |
| `spell-check-differential` | Check spelling in pull request changes. |
| `spell-check-daily` | Check spelling throughout the repository daily or on manual dispatch. |
| `github-release` | Create or update a draft release for a numeric `major.minor.patch` tag, or an existing tag selected manually. |

Dependabot checks GitHub Actions versions monthly.

## Package validation

Each ROS distribution resolves the latest Autoware release with a published Core development image. The container supplies the built Autoware underlay, so this repository does not need a `build_depends.repos` file. Humble validation is independent of which distributions register this package in the Autoware Index.

The matrix keeps Humble and Jazzy independent: one failure does not cancel the other job. Compiler caches are saved only on pushes to `ros2`. Package tests include the existing ament lint suite. Agnocast remains disabled by default; these jobs do not validate the optional Agnocast integration or physical IMU hardware.

Run the repository file checks locally with:

```bash
pre-commit run --all-files
```

C++ and CMake style are checked by ament during package validation. The repository-owned pre-commit configuration avoids introducing conflicting clang-format or cpplint settings.

The validation workflow checks out the repository under `src/tamagawa_imu_driver` in a separate colcon workspace. Build products and downloaded CI tools stay outside the package, so the standard `ament_lint_auto` checks discover maintained files automatically.

Validation calls the shared `autowarefoundation/autoware-index-github-actions` workflow on `main`.

The contribution notice keeps the bare code fence required by `ament_copyright`; `CONTRIBUTING.md` disables only Markdown's code-fence language rule for that reason.

## Maintenance

These workflows and formatting settings are based on the Livox tag filter repository and [Autoware templates](https://github.com/autowarefoundation/sync-file-templates), but are maintained in this repository. There is no automated file synchronization.

The release workflow uses `GITHUB_TOKEN` with `contents: write` and leaves existing published releases unchanged. The other checks need no custom secrets.
