# AGENTS

## Project overview

This repository is a C++ library for processing and visualizing ColoRadar datasets, with Python bindings for downstream workflows. The core implementation lives under `src/` and `include/`; examples and Docker helpers live under `examples/`; and dataset assets are expected under `data/`.

Key docs:
- [README.md](README.md)
- [examples/README.md](examples/README.md)
- [CMakeLists.txt](CMakeLists.txt)

## Working conventions

- Prefer the existing CMake-based build flow over ad hoc scripts.
- Keep C++ changes aligned with the library structure: config code in `src/configs/`, dataset logic in `src/dataset/`, runtime logic in `src/run/`, utilities in `src/utils/`, and Python bindings in `src/python_bindings/`.
- Respect the repository’s C++20 target and the platform-specific compiler setup defined in [CMakeLists.txt](CMakeLists.txt): macOS uses Clang, Linux prefers GCC 11/12.
- When adding new library exports or Python-facing API surface, update the corresponding binding source and keep the Python import names consistent with the build outputs.
- Do not duplicate existing documentation. Link back to [README.md](README.md) and [examples/README.md](examples/README.md) for setup and examples instead of repeating their content.

## Build and validation commands

Run the smallest relevant validation command for changes:

- Local C++ build:
  ```bash
  mkdir -p build && cd build && cmake .. && make -j$(nproc)
  ```
- Project test binary:
  ```bash
  ./build/coloradar_tests
  ```
- Python binding smoke test:
  ```bash
  python3 tests/test_bindings.py
  ```
- Docker example workflow:
  ```bash
  cd examples && docker compose up --build jupyter
  ```

## Environment and dependency notes

- This project is designed around Dockerized builds and local builds with the dependencies listed in [README.md](README.md): Boost, MPI, Eigen, VTK, PCL, OpenCV, octomap, yaml-cpp, jsoncpp, HDF5, PyBind11, and GTest.
- CUDA support is conditional and controlled by the `CUDA_ENV` environment variable and the CMake `CUDA_FOUND` checks in [CMakeLists.txt](CMakeLists.txt).
- If a change affects dataset usage or example scripts, check the assumptions documented in [examples/README.md](examples/README.md) before editing container paths, dataset mounts, or Jupyter setup.

## When making changes

- Prefer targeted edits in the relevant module and keep API or behavior changes consistent with the library’s existing namespacing and patterns.
- If a fix touches examples, Docker config, or dataset assumptions, verify that the command still matches the documented workflow in [README.md](README.md) or [examples/README.md](examples/README.md).
- For repo-wide or architecture-level changes, update the docs and examples together when needed, but keep the instructions minimal and practical.

## Typical task guidance

- For C++ library changes: inspect the headers and source under `include/` and `src/`, then validate with the local build and test binary.
- For Python binding changes: inspect `src/python_bindings/` plus the binding smoke test, then validate with `python3 tests/test_bindings.py`.
- For example or Docker troubleshooting: start from [examples/README.md](examples/README.md) and the `docker-compose.yaml` file under `examples/`.
