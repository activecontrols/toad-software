# Environment & Setup Scripts (`offline_test/scripts/`)

This directory contains developer automation scripts for provisioning and validating the host build environment for the offline test platform.

---

## 1. `setup_env.sh`
Script: [`setup_env.sh`](setup_env.sh)

An automated setup script that inspects your system and provisions all necessary prerequisites without requiring root (`sudo`) access.

### What It Does:
1. **CMake Inspection & Local Installation**:
   - Checks if `cmake` is installed and available in `PATH`.
   - If missing, downloads modern CMake 3.30.3 for Linux x86_64 and unpacks it to `$HOME/.local/bin`.
2. **vcpkg Bootstrapping**:
   - Checks if `$VCPKG_ROOT` is configured and points to a valid `vcpkg` executable.
   - If missing, clones the official Microsoft `vcpkg` repository into `$HOME/vcpkg` and executes `./bootstrap-vcpkg.sh -disableMetrics`.
3. **Environment Setup Instructions**:
   - Outputs the necessary environment variable exports to add to your shell profile (`~/.bashrc` or `~/.zshrc`).

---

## 2. Usage Instructions

Run the setup script from the repository root or the `offline_test/` directory:

```bash
cd firmware/offline_test
bash scripts/setup_env.sh
```

### Persisting Environment Variables
Add the following lines to your `~/.bashrc` or `~/.zshrc`:

```bash
export PATH="$HOME/.local/bin:$PATH"
export VCPKG_ROOT="$HOME/vcpkg"
```

Then reload your shell:
```bash
source ~/.bashrc
```

---

## 3. Verifying the Setup

Verify that CMake and vcpkg are detected:

```bash
cmake --version
# Should report CMake 3.25 or newer

$VCPKG_ROOT/vcpkg version
# Should report vcpkg package management program version
```

