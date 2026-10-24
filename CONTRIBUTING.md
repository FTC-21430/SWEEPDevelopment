# Contributing to SWEEPDevelopment

This repository combines the FTC Robot Controller project with the SWEEP path
library. Keep changes focused and preserve the FTC SDK integration while
improving the team-owned `SWEEP-core` and `TeamCode` modules.

## Before opening a pull request

- Follow the Java style and unit conventions documented in the source and the
  [SWEEP-core guide](SWEEP-core/README.md#units-and-coordinate-conventions).
- Add Javadocs to Java methods and constructors. Document non-obvious units and
  behavior; use inline comments only to explain meaningful reasoning.
- Do not leave commented-out code or TODO comments in a pull request targeting
  `master`. Record unfinished work as a GitHub issue instead.
- Avoid unrelated formatting changes so reviewers can inspect functional
  changes line by line.
- Have at least one other person review the pull request before merging to
  `master`.
- Pull request titles should be descriptive Title Case. Keep the description
  concise, explain behavior changes and risks, and fact-check AI-assisted text.

## Branches and commits

Use descriptive `snake_case` branch names, for example:

```text
fix_action_lifecycle
add_offset_spline_tests
```

Make small commits with imperative, descriptive messages. Do not rewrite shared
history or amend commits unless the team has explicitly agreed to it.

## Local validation

Run the smallest checks covering your change. For path-planning or lifecycle
changes, run:

```sh
./gradlew :SWEEP-core:testDebugUnitTest
```

For TeamCode changes, also run:

```sh
./gradlew :TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac
```

For a broader pre-release check:

```sh
./gradlew :SWEEP-core:testDebugUnitTest :TeamCode:testDebugUnitTest \
  :TeamCode:compileDebugJavaWithJavac :SWEEP-core:assembleRelease
```

Some Android builds require a local root `gradle.properties` containing SDK
configuration such as AndroidX and non-transitive `R` settings. Keep machine-
specific properties and credentials out of version control. GitHub Packages
publishing credentials must be supplied locally or by CI; never put tokens in
Gradle files or commits.

## Pull request workflow

1. Update your local `master`, then create a focused `snake_case` branch.
2. Make the code and test changes, with one logical concern per commit where
   practical.
3. Run relevant tests, compilation, and `git diff --check`; review the diff for
   accidental generated files, secrets, commented-out code, and TODOs.
4. Push the branch and open a descriptive Title Case pull request. Explain the
   problem, approach, test evidence, and any hardware validation still needed.
5. Ask a teammate for review. Address review feedback in new focused commits.
6. Before merging, update from `master` and resolve conflicts without
   discarding another contributor's work. Merge only after approval and passing
   checks.

For robot-motion changes, document the units and test assumptions, and do not
present simulation or unit-test results as a substitute for careful,
low-speed hardware testing.
