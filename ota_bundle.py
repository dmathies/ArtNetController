from pathlib import Path
import struct
import subprocess
import json
import csv

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


def _merged_install_path(build_dir: Path) -> Path:
    return build_dir / "merged-install.bin"


def _manifest_path(build_dir: Path) -> Path:
    return build_dir / "esp-web-tools-manifest.json"


def _bootloader_path(build_dir: Path) -> Path:
    return build_dir / "bootloader.bin"


def _partitions_path(build_dir: Path) -> Path:
    return build_dir / "partitions.bin"


def _esptool_command() -> list[str]:
    package_dir = env.PioPlatform().get_package_dir("tool-esptoolpy")
    if package_dir:
        candidate = Path(package_dir) / "esptool.py"
        if candidate.exists():
            return [env.subst("$PYTHONEXE"), str(candidate)]
    return [env.subst("$PYTHONEXE"), "-m", "esptool"]


def _boot_app0_path() -> Path | None:
    framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
    if not framework_dir:
        return None
    candidate = Path(framework_dir) / "tools" / "partitions" / "boot_app0.bin"
    return candidate if candidate.exists() else None


def _chip_family() -> str:
    mcu = str(env.BoardConfig().get("build.mcu", "")).lower()
    mapping = {
        "esp32": "ESP32",
        "esp32s2": "ESP32-S2",
        "esp32s3": "ESP32-S3",
        "esp32c2": "ESP32-C2",
        "esp32c3": "ESP32-C3",
        "esp32c5": "ESP32-C5",
        "esp32c6": "ESP32-C6",
        "esp32h2": "ESP32-H2",
        "esp32p4": "ESP32-P4",
    }
    return mapping.get(mcu, "ESP32")


def _parse_int(value: str) -> int:
    value = (value or "").strip()
    if not value:
        return 0
    return int(value, 0)


def _filesystem_offset() -> int | None:
    project_dir = Path(env.subst("$PROJECT_DIR"))
    partition_rel = env.subst("$BOARD_BUILD_PARTITIONS")
    partition_csv = project_dir / partition_rel
    if not partition_csv.exists():
        return None

    with partition_csv.open("r", encoding="utf-8") as infile:
        reader = csv.reader(row for row in infile if row.strip() and not row.lstrip().startswith("#"))
        for row in reader:
            if len(row) < 5:
                continue
            name = row[0].strip().lower()
            part_type = row[1].strip().lower()
            subtype = row[2].strip().lower()
            if part_type == "data" and subtype in ("spiffs", "littlefs"):
                return _parse_int(row[3])
            if name in ("spiffs", "littlefs", "fs"):
                return _parse_int(row[3])
    return None


def _bundle_is_fresh(bundle_path: Path, firmware_path: Path, filesystem_path: Path) -> bool:
    if not bundle_path.exists():
        return False
    bundle_mtime = bundle_path.stat().st_mtime
    return bundle_mtime >= firmware_path.stat().st_mtime and bundle_mtime >= filesystem_path.stat().st_mtime


def _web_install_outputs_are_fresh(merged_path: Path, manifest_path: Path, inputs: list[Path]) -> bool:
    if not merged_path.exists() or not manifest_path.exists():
        return False
    output_mtime = min(merged_path.stat().st_mtime, manifest_path.stat().st_mtime)
    return all(output_mtime >= path.stat().st_mtime for path in inputs)


def build_ota_bundle(*_args, **_kwargs):
    build_dir = Path(env.subst("$BUILD_DIR"))
    firmware_path = _firmware_path(build_dir)
    filesystem_path = _filesystem_path(build_dir)
    bundle_path = _bundle_path(build_dir)

    if not firmware_path.exists() or not filesystem_path.exists():
        print(f"[ota-bundle] waiting for {firmware_path.name} and {filesystem_path.name}")
        return

    firmware = firmware_path.read_bytes()
    filesystem = filesystem_path.read_bytes()

    if _bundle_is_fresh(bundle_path, firmware_path, filesystem_path):
        print(f"[ota-bundle] up to date: {bundle_path.name}")
    else:
        payload = HEADER.pack(MAGIC, VERSION, len(firmware), len(filesystem))

        with bundle_path.open("wb") as outfile:
            outfile.write(payload)
            outfile.write(firmware)
            outfile.write(filesystem)

        print(
            f"[ota-bundle] wrote {bundle_path.name} "
            f"({len(firmware)} bytes firmware + {len(filesystem)} bytes filesystem)"
        )


def build_web_installer_artifacts(*_args, **_kwargs):
    build_dir = Path(env.subst("$BUILD_DIR"))
    firmware_path = _firmware_path(build_dir)
    filesystem_path = _filesystem_path(build_dir)
    merged_path = _merged_install_path(build_dir)
    manifest_path = _manifest_path(build_dir)
    bootloader_path = _bootloader_path(build_dir)
    partitions_path = _partitions_path(build_dir)
    boot_app0_path = _boot_app0_path()

    if not firmware_path.exists() or not filesystem_path.exists():
        print(f"[esp-web-tools] waiting for {firmware_path.name} and {filesystem_path.name}")
        return

    web_inputs = [firmware_path, filesystem_path, bootloader_path, partitions_path]
    if boot_app0_path is None or not boot_app0_path.exists():
        print("[esp-web-tools] boot_app0.bin not found; skipping manifest generation")
        return
    web_inputs.append(boot_app0_path)

    if not all(path.exists() for path in web_inputs):
        print("[esp-web-tools] waiting for bootloader.bin, partitions.bin, firmware.bin, and littlefs.bin")
        return

    fs_offset = _filesystem_offset()
    if fs_offset is None:
        print("[esp-web-tools] filesystem offset not found in partition CSV; skipping manifest generation")
        return

    if _web_install_outputs_are_fresh(merged_path, manifest_path, web_inputs):
        print(f"[esp-web-tools] up to date: {merged_path.name}, {manifest_path.name}")
        return

    esptool_cmd = _esptool_command()
    subprocess.check_call(
        esptool_cmd
        + [
            "--chip",
            str(env.BoardConfig().get("build.mcu", "esp32")),
            "merge_bin",
            "-o",
            str(merged_path),
            "--flash_mode",
            "dio",
            "--flash_freq",
            "40m",
            "--flash_size",
            env.BoardConfig().get("upload.flash_size", "4MB"),
            "0x1000",
            str(bootloader_path),
            "0x8000",
            str(partitions_path),
            "0xe000",
            str(boot_app0_path),
            "0x10000",
            str(firmware_path),
        ],
        cwd=env.subst("$PROJECT_DIR"),
    )

    manifest = {
        "name": f"CableCar ({env.subst('$PIOENV')})",
        "version": env.GetProjectOption("custom_firmware_version", "dev"),
        "new_install_prompt_erase": True,
        "builds": [
            {
                "chipFamily": _chip_family(),
                "parts": [
                    {"path": merged_path.name, "offset": 0},
                    {"path": filesystem_path.name, "offset": fs_offset},
                ],
            }
        ],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(
        f"[esp-web-tools] wrote {merged_path.name} and {manifest_path.name} "
        f"(filesystem offset 0x{fs_offset:x})"
    )


firmware_target = env.subst("$BUILD_DIR/${PROGNAME}.bin")
fs_image_target = env.DataToBin(
    env.subst("$BUILD_DIR/${ESP32_FS_IMAGE_NAME}"),
    env.subst("$PROJECT_DATA_DIR"),
)
env.NoCache(fs_image_target)
AlwaysBuild(fs_image_target)

env.AddCustomTarget(
    name="buildota",
    dependencies=[firmware_target, fs_image_target],
    actions=[build_ota_bundle, build_web_installer_artifacts],
    title="Build OTA bundle",
    description="Create OTA bundle plus ESP Web Tools installer artifacts",
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
