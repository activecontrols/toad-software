# TOAD Software

Firmware and ground station software for PSP's TOAD liquid-fueled lander vehicle.

## Development Enviroment Setup

Install VSCode from https://code.visualstudio.com/ and clone this repository with `git clone https://github.com/activecontrols/toad-software.git`.

Use `File -> Open Workspace from File` to open the [toad-software.code-workspace](toad-software.code-workspace) file as a VSCode multiroot workspace. This essentially allows multiple projects with their own VSCode settings to be open simultaneously, which is useful for managing the firmware and UI projects.

### Firmware

We use [PlatformIO](https://platformio.org/) to manage building and uploading firmware. Install the VSCode extension (https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide), and PlatformIO will handle the rest of the setup and configuration upon opening the project.

In order to handle multiple boards, we have defined individual PlatformIO enviroments for each board on TOAD, which can be built independently from the PlatformIO sidebar.

TOAD uses custom PCBs, these are defined in the [firmware/boards](firmware/boards/) folder and documented at https://purdue-space-program.atlassian.net/wiki/spaces/PAC/pages/2045575171/Configuring+a+custom+board+with+PlatformIO

#### FC / EC Shared Codebase

The FC and EC projects are built from the same codebase, using a few rules to define the binary for each. First, each has their own `main.cpp`, as [firmware/src/fc_main.cpp](firmware/src/fc_main.cpp) and [firmware/src/ec_main.cpp](firmware/src/ec_main.cpp). These main files pull in the modules each board needs, so having a [firmware/lib/](firmware/lib/) folder that is only used on one project is fine. Additional control is provided through the `TOAD_FLIGHT_CONTROLLER_ONLY` and `TOAD_ENGINE_CONTROLLER_ONLY` flags, which can be used (sparingly) with `#ifdef`. This setup is controlled in [firmware/platformio.ini](firmware/platformio.ini). The active enviroment can be selected from the VSCode lower toolbar, which will say `Default (firwmare)` by default, but can be customized to view the project with the specific compiler flags for the FC or EC.

The STM32G4 codebase is located in [firmware/src/prog_main.cpp](firmware/src/prog_main.cpp). This is a much smaller codebase and makes much less use of the shared libraries.

For debugging, the enviroment (board) should be selected from the VSCode lower toolbar.

### UI / Groundstation

Just want the latest .exe? - go to https://github.com/activecontrols/toad-software/actions/workflows/ui_build.yml

#### Setting up gcc on Windows
Follow the instructions here, including updating PATH: https://code.visualstudio.com/docs/cpp/config-mingw

mingw provides the infrastructure needed to install and run linux tools natively on windows. (Note: as opposed to WSL, which isn't useful here because we are building a windows executable).

After restarting VS Code, gcc, gdb, and g++ should work from the VS Code terminal.

Running `mingw32-make -f UI.mk` in the toad-software folder should build the project. `mingw32-make` can be installed with `pacman -S --needed base-devel mingw-w64-ucrt-x86_64-toolchain` in the MinGW terminal.  

#### Intellisense
Intellisense should "just work" once you have mingw installed (along with https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools). If it doesn't, make sure that you can run g++ from the command line. If something isn't working, look at [c_cpp_properties.json](UI/.vscode/c_cpp_properties.json) and the intellisense status in the bottom right of the VSCode window.


## Documentation

For details on high level architecture decisions, see [docs/](docs/). Individual functions and modules are documented in-line.

## Contributing


### Features

For new features, create a branch of with the name `feature/quick_description` such as `feature/add_imu_support`. Work on code in this branch instead of working directly in the main branch. When you are ready to merge your feature in, talk to a GNC lead to review your changes. Then run the following comamands:

```
git fetch
git rebase origin/main
git push --force-with-lease

# Stop here and test/review the changes again to make sure the rebase didn't break anything

git checkout main
git merge feature/X --ff-only
git push
```

These steps begin by rebasing the feature branch onto the latest commit from main. Rebasing rewrites the branch history so the feature changes appear as though they were created after the most recent commit to main. This helps ensure that merge conflicts don' break the new feature as the code should be re-tested after the rebase.

During the rebase, conflicts may need to be resolved or additional fixes may be required to restore functionality. Handling those issues before merging helps ensure that merges into main remain clean and linear.

A force push is required after the rebase because rebasing changes the commit history of the feature branch. Be careful with force pushing, as it is one of the few potentially destructive git operations. Only ever force push to a feature branch, and ensure that you are on the latest remote commit before doing so.

Commits should generally by single sub-features and have a short, descriptive message. `improve performance by fixing SPI clock speed` is great, but `quick fixes` or similar make it hard to understand the reasoning for changes. Optionally, commits can be squashed before merging to main. Git is an incredibly useful tool, when in doubt, commit to avoid losing work.

Please delete branches after merging.

If your change is not fully tested before merging, make a note in `PendingVerification.md` to track the additional testing work. Adding `// TODO - short desc` as appropriate in the code is also helpful, and is mandatory if a change disables functionality that must be re-enabled before flight, such as a safety check that must be bypassed for testing.

### Bugfixes

For new features, create a branch of with the name `bugfix/quick_description` such as `bugfix/fix_imu_axes`. Follow the same merging conventions as above. For very simple fixes (such as changing a single value), a commit directly to main is acceptable.

### Test Day Branches

Every test day, a branch should be created of the form `test/type_month_day` such as `test/ec_waterflow_aug_24`. Test branches allow quick changes to be made and saved without worrying about the formal merge process on main. After the test day concludes, the entire branch may be abandoned, merged, or individual commits may be cherry-picked into main.

In general, commit before or after every test event so that data logs can be compared against the code used to run that test. Test day branches should only be deleted once all changes have been merged/ignored or another test of the same type supersedes the branch entirely. Test branches are often useful to figure out why something used to work, so in general they should be kept.

