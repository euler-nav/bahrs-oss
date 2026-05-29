## Introduction

This repository is a version-controlled collection of reusable software components (SWCs) designed for projects like [bahrs-sw](https://bitbucket.org/fedor_baklanov/bahrs-sw/src).

**Core Concepts:**

-   **One Branch Per Component:** Each SWC (e.g., a driver or a library) is maintained on its own dedicated branch.
-   **Integration via Submodules:** Projects consume these SWCs as Git submodules, pinned to specific version tags for stability.
-   **Develop in Place:** All development and testing of a component occurs within the host project that uses it. This ensures the component works in a real-world context.

> **Important:** The source code in this repository is not intended for standalone development. It requires a host project to provide the build system and hardware context.

### Our Rationale

This model is a deliberate choice to solve the challenge of supporting multiple hardware versions (e.g., BAHRS V2, BAHRS V3) simultaneously.

**Key Benefits:**

1.  **Preserves Stability:** It allows us to update a SWC for a new project without forcing changes upon older, validated projects. A project for BAHRS V2 can continue using a verified version of a driver, while a new project for BAHRS V3 can use an updated one.
2.  **Ensures Reproducibility:** Pinning submodules to tags guarantees that we can always rebuild an exact version of a project with its original dependencies.
3.  **Avoids Infrastructure Overhead:** By using Git's built-in features, we avoid the need to maintain a separate package management system like Artifactory.

## Repository Structure

- **Component branches**: Each software component lives on its own branch named `swc/<component-name>` (e.g., `swc/bmp384-driver`)
- **Feature branches**: Temporary branches for development named `feature/<description>` (e.g., `feature/update-bmp384-driver`)
- **Tags**: Release versions use semantic versioning with component prefix (e.g., `BMP384_DRIVER_V_1_0_HW_V2`)

## How to Modify an Existing Software Component

**Overview**: You'll work in two repositories simultaneously - this component library and the project that uses it.

### Phase 1: Setup Development Branches

1. **Create SWC feature branch** in this repository:
   
    - Create a feature branch from the component branch you want to modify
    - Example: create `feature/update-bmp384-driver` from `swc/bmp384-driver`
    - We'll call this the "SWC feature branch"

2. **Create project feature branch** in the consuming project repository:
   
    - Create a feature branch in the project that uses this component
    - Example: create `feature/integrate-bmp384-updates` in bahrs-sw repository

### Phase 2: Development Work

1. **Setup your workspace**:
   
    - Checkout the project feature branch in your consuming project
    - Navigate to the submodule location (where this component is referenced)
    - Checkout the SWC feature branch within the submodule
    **Terminal commands:**
    ```bash
    # In the consuming project (e.g., bahrs-sw)
    git checkout feature/integrate-bmp384-updates
    cd path/to/submodule/location
    git checkout feature/update-bmp384-driver
    ```

    **TortoiseGit users:** Navigate to the submodule folder and use the GUI to checkout the desired branch.

2. **Make your changes**:
   
    - Modify the component code to work in the target project
    - Build and test within the consuming project to ensure it works
    - **Important**: All commits during development go to the SWC feature branch

3. **Commit component changes**:
   
    - Navigate to the submodule location
    - Use standard git workflow to commit changes to the SWC feature branch
    - Push changes to upstream
    ```bash
    # Inside the submodule directory
    git add .
    git commit -m "Update BMP384 driver for new hardware support"
    git push origin feature/update-bmp384-driver
    ```

4. **Update project to reference new submodule commit**:
   
    - Navigate back to the project root directory
    - Commit the submodule update (this records the new commit SHA)
    - Push to upstream
    ```bash
    # Back in project root
    cd ../..  # or however many levels up to get to project root
    git add path/to/submodule/location
    git commit -m "Update BMP384 driver submodule to latest development version"
    git push origin feature/integrate-bmp384-updates
    ```

### Phase 3: Release Process

1. **Create pull request for component**:
   
    - Raise a PR from the SWC feature branch to the main component branch
    - Example: PR from `feature/update-bmp384-driver` to `swc/bmp384-driver`

2. **Tag the release** (after PR is merged):
   
    - Create a tag for the merge commit on the component branch
    - Tag name should indicate compatible projects/hardware versions
    - Example: `BMP384_DRIVER_V_1_0_HW_V2`
    - Use semantic versioning: `<COMPONENT>_V_<MAJOR>_<MINOR>_<TARGET>`

3. **Pin project to release tag**:
   
    - In the project repository, navigate to the submodule location
    - Checkout the newly created tag (this puts submodule in detached HEAD state)
    - Navigate back to project root and commit the submodule update
    ```bash
    # In project submodule location
    git fetch --tags
    git checkout BMP384_DRIVER_V_1_0_HW_V2
    # Back to project root
    cd ../..
    git add path/to/submodule/location
    git commit -m "Pin BMP384 driver to release BMP384_DRIVER_V_1_0_HW_V2"
    ```

4. **Finalize project integration**:

    - Push changes to upstream
    - Raise a PR from project feature branch to development branch

## Key Points for New Developers

- **Two repositories, two branches**: You're always working with a feature branch in both the component library AND the consuming project
- **Submodule commits first**: Always commit and push submodule changes before committing the parent project
- **Tags are releases**: Only pin submodules to tags in production; use feature branches during development
- **Detached HEAD is normal**: When checking out tags in submodules, Git will warn about "detached HEAD" - this is expected and correct

