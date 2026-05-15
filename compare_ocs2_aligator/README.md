# aligator analytic vs OCS2 CppADCodeGen benchmark

This small standalone benchmark compares two derivative paths for the same
kinodynamics flow map:

- aligator analytical derivatives: `KinodynamicsFwdDynamicsTpl<double>::dForward()`
- OCS2 CppADCodeGen: `CppAdInterface::createModels()/loadModelsIfAvailable()` and `getJacobian()`

The cleanest apples-to-apples comparison is for input-side derivatives
(`u`, `wrench`, `acc`, `left_wrench`, `right_wrench`), because both backends
use the same Euclidean input coordinates. State-side derivatives are reported
too, but their consistency check can include convention differences between
aligator's manifold/tangent `Jx` and CppAD's explicit Euler-coordinate state
perturbation.

Build:

```bash
cd /home/songchao/OPTControl_env/OpenLoong-Dyn-Control/compare_ocs2_aligator
cmake -S . -B build-cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build-cmake -j
```

Run a quick benchmark:

```bash
./build/compare_derivatives --iters 200 --warmup 20 --wrt xu --recompile
```

Useful options:

- `--wrt xu`: full state and input Jacobian.
- `--wrt x`, `--wrt u`, `--wrt q`, `--wrt v`, `--wrt wrench`, `--wrt acc`, `--wrt left_wrench`, `--wrt right_wrench`.
- `--recompile`: force CppADCodeGen to regenerate and compile the dynamic library.
- `--codegen-dir PATH`: where generated CppADCodeGen libraries are stored.
- `--urdf PATH`: robot URDF. Defaults to `../models/g1/g1_29dof.urdf`.

The benchmark prints CppAD prepare time separately from repeated Jacobian query
time, so the one-time code generation cost does not get mixed with runtime MPC
linearization cost.
