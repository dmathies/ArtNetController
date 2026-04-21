from pathlib import Path
import gzip
import shutil

from SCons.Script import DefaultEnvironment


env = DefaultEnvironment()


def _compress_if_stale(src: Path) -> bool:
    dst = src.with_name(src.name + ".gz")
    if dst.exists() and dst.stat().st_mtime >= src.stat().st_mtime:
        return False

    with src.open("rb") as infile, gzip.open(dst, "wb", compresslevel=9) as outfile:
        shutil.copyfileobj(infile, outfile)
    return True


def gzip_web_assets(*_args, **_kwargs):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    web_root = project_dir / "data" / "wifi-manager"
    changed = []

    for src in sorted(web_root.glob("*.html")):
        if _compress_if_stale(src):
            changed.append(src.name + ".gz")

    if changed:
        print("[gzip] refreshed " + ", ".join(changed))


gzip_web_assets()
