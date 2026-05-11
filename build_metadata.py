from pathlib import Path
import subprocess
from datetime import datetime, timezone

from SCons.Script import DefaultEnvironment


env = DefaultEnvironment()


def _get_git_value(args, default):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    try:
        return subprocess.check_output(
            args,
            cwd=project_dir,
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        return default


def _escape_cpp(value: str) -> str:
    return value.replace("\\", "\\\\").replace('"', '\\"')


def generate_build_metadata(*_args, **_kwargs):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    include_dir = project_dir / "include"
    header_path = include_dir / "BuildMetadata.h"
    include_dir.mkdir(parents=True, exist_ok=True)

    version = env.GetProjectOption("custom_firmware_version", "dev")
    build_env = env.subst("$PIOENV")
    git_sha = _get_git_value(["git", "rev-parse", "--short", "HEAD"], "unknown")
    git_tag = _get_git_value(["git", "describe", "--tags", "--always", "--dirty"], git_sha)
    build_time = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    build_summary = f"{version} ({git_sha})"

    header = f"""#pragma once

#define APP_BUILD_VERSION "{_escape_cpp(version)}"
#define APP_BUILD_GIT_SHA "{_escape_cpp(git_sha)}"
#define APP_BUILD_GIT_REF "{_escape_cpp(git_tag)}"
#define APP_BUILD_ENV "{_escape_cpp(build_env)}"
#define APP_BUILD_TIME_UTC "{_escape_cpp(build_time)}"
#define APP_BUILD_SUMMARY "{_escape_cpp(build_summary)}"
"""

    if not header_path.exists() or header_path.read_text(encoding="utf-8") != header:
        header_path.write_text(header, encoding="utf-8")
        print(f"[build-meta] wrote {header_path.name}: {build_summary} {build_env}")
    else:
        print(f"[build-meta] up to date: {header_path.name}")


generate_build_metadata()
