# Silk

Silk is a collection of robotic UI experiments. Check out the [live gallery](http://djbutler.github.io/silk/).

To make deployment to GitHub pages easier, all the code lives in the `gh-pages` branch.

## Ceres Solver in the browser

One technique used extensively in Silk is mathematical optimization, especially for inverse kinematics (IK). Silk's optimization code is written in C++ and uses [Ceres Solver](http://www.ceres-solver.org/), a really nice non-linear least squares library with automatic differentiation. The C++ code is then transpiled into Javascript using [Emscripten](http://kripken.github.io/emscripten-site/).
