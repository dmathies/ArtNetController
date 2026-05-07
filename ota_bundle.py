from pathlib import Path
import struct
import subprocess

from SCons.Script import AlwaysBuild, DefaultEnvironment


env = DefaultEnvironment()

MAGIC = b"CCAROTA1"
VERSION = 1
HEADER = struct.Struct("<8sIII")


def _firmware_path(build_dir: Path) -> Path:
    return build_dir / f"{env.subst('$PROGNAME')}.bin"


def _filesystem_path(build_dir: Path) -> Path:
    for name in ("littlefs.bin", "spiffs.bin"):
        candidate = build_dir / name
        if candidate.exists():
            return candidate
    return build_dir / "littlefs.bin"


def _bundle_path(build_dir: Path) -> Path:
    return build_dir / "ota_bundle.ota"


def _bundle_is_fresh(bundle_path: Path, firmware_path: Path, filesystem_path: Path) -> bool:
    if not bundle_path.exists():
        return False
    bundle_mtime = bundle_path.stat().st_mtime
    return bundle_mtime >= firmware_path.stat().st_mtime and bundle_mtime >= filesystem_path.stat().st_mtime


def build_ota_bundle(*_args, **_kwargs):
    build_dir = Path(env.subst("$BUILD_DIR"))
    firmware_path = _firmware_path(build_dir)
    filesystem_path = _filesystem_path(build_dir)
    bundle_path = _bundle_path(build_dir)

    if not firmware_path.exists() or not filesystem_path.exists():
        print(f"[ota-bundle] waiting for {firmware_path.name} and {filesystem_path.name}")
        return

    if _bundle_is_fresh(bundle_path, firmware_path, filesystem_path):
        print(f"[ota-bundle] up to date: {bundle_path.name}")
        return

    firmware = firmware_path.read_bytes()
    filesystem = filesystem_path.read_bytes()
    payload = HEADER.pack(MAGIC, VERSION, len(firmware), len(filesystem))

    with bundle_path.open("wb") as outfile:
        outfile.write(payload)
        outfile.write(firmware)
        outfile.write(filesystem)

    print(
        f"[ota-bundle] wrote {bundle_path.name} "
        f"({len(firmware)} bytes firmware + {len(filesystem)} bytes filesystem)"
    )


def build_ota_target(*_args, **_kwargs):
    # Force both artifacts to be rebuilt when users invoke `-t buildota`,
    # so filesystem-only UI changes are always included in the final bundle.
    python_exe = env.subst("$PYTHONEXE")
    project_dir = env.subst("$PROJECT_DIR")
    pio_env = env.subst("$PIOENV")

    subprocess.check_call(
        [python_exe, "-m", "platformio", "run", "-d", project_dir, "-e", pio_env],
        cwd=project_dir,
    )
    subprocess.check_call(
        [python_exe, "-m", "platformio", "run", "-d", project_dir, "-e", pio_env, "-t", "buildfs"],
        cwd=project_dir,
    )
    build_ota_bundle()


firmware_target = env.subst("$BUILD_DIR/${PROGNAME}.bin")
fs_image_target = env.DataToBin(
    env.subst("$BUILD_DIR/${ESP32_FS_IMAGE_NAME}"),
    env.subst("$PROJECT_DATA_DIR"),
)
env.NoCache(fs_image_target)
AlwaysBuild(fs_image_target)

env.AddPostAction(firmware_target, build_ota_bundle)
env.AddPostAction(fs_image_target, build_ota_bundle)
env.AddCustomTarget(
    name="buildota",
    dependencies=None,
    actions=[build_ota_target],
    title="Build OTA bundle",
    description="Create a combined firmware + LittleFS OTA bundle",
)


def build_all_ota_bundles(*_args, **_kwargs):
    python_exe = env.subst("$PYTHONEXE")
    project_dir = env.subst("$PROJECT_DIR")
    command = (
        f'"{python_exe}" -m platformio run '
        f'-e seeed_xiao_esp32s3 '
        f'-e seeed_xiao_esp32s3_led '
        f'-e seeed_xiao_esp32s3_relay '
        f'-t buildota'
    )
    env.Execute(command)


if env.subst("$PIOENV") == "seeed_xiao_esp32s3":
    env.AddCustomTarget(
        name="buildota-all",
        dependencies=None,
        actions=[build_all_ota_bundles],
        title="Build all OTA bundles",
        description="Create combined OTA bundles for BLDC, LED, and relay variants",
    )
