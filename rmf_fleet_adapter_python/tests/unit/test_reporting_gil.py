"""Regression tests for GIL release during Open-RMF reporting calls."""

import os
from pathlib import Path
import subprocess
import sys

import pytest


SCENARIO = Path(__file__).with_name("reporting_gil_scenario.py")


@pytest.mark.parametrize(
    "method_name",
    ["log_info", "log_warning", "log_error", "create_issue"],
)
def test_reporting_releases_gil(method_name):
    result = subprocess.run(
        [sys.executable, "-u", str(SCENARIO), method_name],
        env=os.environ.copy(),
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )

    assert result.returncode == 0, (
        f"{method_name} failed with exit code {result.returncode}\n"
        f"stdout:\n{result.stdout}\n"
        f"stderr:\n{result.stderr}"
    )
