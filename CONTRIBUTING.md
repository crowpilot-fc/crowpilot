<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->

# Contributing to CrowPilot

Thank you for your interest in CrowPilot.

## A note on safety

CrowPilot controls aircraft. Code that ends up in `main` runs on real hardware that can hurt people. Before contributing safety-critical code (control loop, mixer, failsafe, actuator output, IMU, RX), read [SAFETY.md](SAFETY.md) and the relevant sections of the spec. Be conservative. When in doubt, default to "stop the motors".

## License

CrowPilot is released under the GNU General Public License v3.0 or later. Documentation is released under Creative Commons Attribution-ShareAlike 4.0 International (CC-BY-SA-4.0). By contributing to this project, you agree that your contributions will be licensed under the same terms.

## Contributor License Agreement (CLA)

CrowPilot uses a Contributor License Agreement based on the Project Harmony Individual CLA (HA-CLA-I) with the "any license" outbound option selected. The CLA does **not** transfer your copyright. You retain ownership of your contribution. The CLA grants the project maintainer a perpetual license to redistribute your contribution, including the right to relicense under different terms in the future.

The first time you open a pull request, the CLA Assistant bot will request that you sign the CLA electronically via GitHub OAuth. After signing once, all future contributions from the same GitHub account are covered automatically.

The full CLA text lives at [CLA.md](CLA.md).

## Developer Certificate of Origin (DCO)

In addition to the CLA, every commit must carry a Developer Certificate of Origin sign-off. Add `-s` to your git commit command:

    git commit -s -m "your message"

This adds a `Signed-off-by:` line to the commit message. By doing so, you certify that you have the right to submit the work under the project's license. Read the full DCO text at https://developercertificate.org/.

## Code style

- All source files must include the SPDX header and copyright line (see [License headers](#license-headers) below).
- Follow the existing naming conventions: `lower_snake_case` variables, `lowerCamelCase` functions, `UpperCamelCase` types, `UPPER_SNAKE_CASE` constants.
- Default to no comments unless the *why* of the code is non-obvious.

## License headers

Every source file must begin with:

```cpp
// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Nitin Kumar
```

Every documentation file must begin with:

```markdown
<!-- SPDX-License-Identifier: CC-BY-SA-4.0 -->
<!-- Copyright (C) 2026 Nitin Kumar -->
```

YAML, CMake, and shell files use the `# SPDX-...` comment style.

If you contribute a file with substantial new code, you may add an additional copyright line beneath the existing one for your own copyright. Do not modify or remove the existing maintainer copyright.

## Pull request process

1. Fork the repository.
2. Create a feature branch.
3. Make your changes. Sign commits with `-s`.
4. Open a pull request against `main`.
5. The CLA Assistant will request signature if this is your first PR.
6. CI must pass (build + lint).
7. The maintainer reviews and merges.

## Questions

Open a GitHub Discussion for questions about the project. Open a GitHub Issue for bug reports and feature requests.
