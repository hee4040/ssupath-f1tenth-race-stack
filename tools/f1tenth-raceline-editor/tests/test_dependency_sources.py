from pathlib import Path


TPH_REV = "aa950f6045680366b789dbb855db8d59d54b1db5"
TPH_GIT = "https://github.com/TUMFTM/trajectory_planning_helpers.git"


def test_tph_080_is_pinned_to_upstream_git_commit() -> None:
    pyproject = Path("pyproject.toml").read_text(encoding="utf-8")

    # Keep this test dependency-free: the project intentionally targets Python 3.10,
    # where stdlib tomllib is not available.
    assert '"trajectory-planning-helpers==0.80"' in pyproject
    assert "[tool.uv.sources]" in pyproject
    assert (
        "trajectory-planning-helpers = { "
        f'git = "{TPH_GIT}", rev = "{TPH_REV}" }}'
    ) in pyproject
