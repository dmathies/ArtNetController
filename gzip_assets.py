from pathlib import Path
import gzip
import shutil

from SCons.Script import DefaultEnvironment


env = DefaultEnvironment()


def _compress(src: Path, dst: Path) -> bool:
    dst.parent.mkdir(parents=True, exist_ok=True)
    with src.open("rb") as infile, gzip.open(dst, "wb", compresslevel=9) as outfile:
        shutil.copyfileobj(infile, outfile)
    return True


def gzip_web_assets(*_args, **_kwargs):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    src_root = project_dir / "web-src" / "wifi-manager"
    dst_root = project_dir / "data" / "wifi-manager"
    changed = []

    for src in sorted(src_root.glob("*.html")):
        dst = dst_root / (src.name + ".gz")
        if _compress(src, dst):
            changed.append(dst.name)

    if changed:
        print("[gzip] refreshed " + ", ".join(changed))


gzip_web_assets()
