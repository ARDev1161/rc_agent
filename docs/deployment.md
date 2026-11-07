# Documentation build and GitHub Pages

The site uses **Sphinx, MyST, Furo, Mermaid, Doxygen, Breathe, Graphviz** — matching the libaudition documentation style.

## Branch isolation

- Documentation sources and Pages workflow: `rc_agent:documentation` **only**.
- C++/ROS2 application code: `rc_agent:feat/rc-api-v2` (later a reviewed stable release branch).
- Documentation branch does not contain a production source tree. GitHub Actions checks out production into a temporary, **untracked** `source/` directory during the build, then generates API HTML/XML and removes it with the runner.
- Never merge `documentation` into `dev`, `master`, or `feat/rc-api-v2`, and never merge C++ sources into `documentation`.
- Generated HTML/XML remains in `docs/_build` and is uploaded directly as the Pages artifact, not committed to either branch.

## Local build

From a checkout of `documentation`:

```sh
git clone --branch documentation https://github.com/ARDev1161/rc_agent.git rc_agent-docs
cd rc_agent-docs
git clone --branch feat/rc-api-v2 --depth 1 https://github.com/ARDev1161/rc_agent.git source
git -C source rev-parse HEAD > docs/_source_sha.txt
python3 -m pip install -r docs/requirements.txt
doxygen docs/Doxyfile
sphinx-build -n -W --keep-going -b html docs docs/_build/html
cp -a docs/_build/doxygen docs/_build/html/doxygen
```

Required system packages: `doxygen`, `graphviz`. Open `docs/_build/html/index.html`.

## Publish

In **Settings → Pages → Build and deployment** select **GitHub Actions**. The workflow `.github/workflows/docs-pages.yml` builds on pushes to `documentation` or manual `workflow_dispatch`, and publishes the generated artifact. Expected URL: https://ardev1161.github.io/rc_agent/ . It will not exist until Pages is enabled and a deployment succeeds.

### Keep documentation fresh

The documentation workflow always checks out the current `feat/rc-api-v2` HEAD and embeds its full commit SHA in the generated title (first 12 characters displayed). Documentation-only changes trigger a new build; after every meaningful code release/migration update, the hourly development automation must also update docs content or a `docs/source-refresh.txt` marker in `documentation` to trigger an updated API build. The marker must contain the **verified** production HEAD SHA, and no production code. A failed build/deployment blocks documentation acceptance. Branch protection/configuration is managed by the repo owner.

Source changes alone do **not** invoke a workflow stored exclusively on this non-default docs branch; the automation is the synchronization bridge. This limitation is intentional to keep documentation workflow out of code branches.
