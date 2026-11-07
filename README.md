# rc_agent documentation

Documentation-only branch for [rc_agent](https://github.com/ARDev1161/rc_agent). The branch contains **no production source tree**; GitHub Actions checks out `feat/rc-api-v2` separately at build time to generate the current C++ API reference.

- [Sphinx sources](docs/index.md)
- [Site build/deployment](docs/deployment.md)
- [Approved implementation plan](https://github.com/ARDev1161/AmurCore/blob/feat/rc-api-v2/PLAN.md)

The website uses the same Sphinx / MyST / Furo / Mermaid / Doxygen / Breathe stack as libaudition. **Do not merge this branch into `dev`, `master` or `feat/rc-api-v2`.** API documentation is generated against a separate checkout of code; documentation authors do not import source files into this branch.

To publish: Settings → Pages → Build and deployment → Source: **GitHub Actions**. Then push a commit to `documentation` or run the Documentation site workflow from that branch.
