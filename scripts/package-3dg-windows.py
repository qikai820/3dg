#!/usr/bin/env python3
"""Stage a self-contained Windows x64 3DG build and create a ZIP archive."""

import argparse
import hashlib
from pathlib import Path
import re
import shutil
import subprocess
import sys
import zipfile


ROOT = Path(__file__).resolve().parents[1]
IMPORT_RE = re.compile(r"^\s*DLL Name:\s*(\S+)", re.MULTILINE)
SYSTEM_DLLS = {
    "advapi32.dll", "bcrypt.dll", "cfgmgr32.dll", "comctl32.dll",
    "comdlg32.dll", "crypt32.dll", "d2d1.dll", "d3d9.dll", "d3d11.dll",
    "dinput8.dll", "dnsapi.dll", "dwmapi.dll", "dwrite.dll",
    "dxgi.dll", "dxva2.dll", "evr.dll", "gdi32.dll", "gdiplus.dll", "imm32.dll",
    "iphlpapi.dll", "kernel32.dll", "mf.dll", "mfplat.dll",
    "mfuuid.dll", "mpr.dll", "msvcrt.dll", "netapi32.dll",
    "normaliz.dll", "ntdll.dll", "ole32.dll", "oleacc.dll",
    "oleaut32.dll", "opengl32.dll", "powrprof.dll", "propsys.dll",
    "psapi.dll", "secur32.dll", "setupapi.dll", "shell32.dll",
    "shlwapi.dll", "strmiids.dll", "user32.dll", "userenv.dll",
    "uuid.dll", "uxtheme.dll", "version.dll", "winmm.dll",
    "winspool.drv", "ws2_32.dll", "wtsapi32.dll",
}


def imports(path: Path, objdump: str) -> set[str]:
    result = subprocess.run(
        [objdump, "-p", str(path)], capture_output=True, text=True, check=True
    )
    return {name.lower() for name in IMPORT_RE.findall(result.stdout)}


def copy(src: Path, dest: Path) -> None:
    if not src.is_file():
        raise FileNotFoundError(src)
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build-windows")
    parser.add_argument("--qt-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/windows")
    parser.add_argument("--objdump", default="x86_64-w64-mingw32-objdump")
    parser.add_argument("--lrelease", default="lrelease")
    args = parser.parse_args()
    build = args.build_dir.resolve()
    qt = args.qt_dir.resolve()
    output = args.output_dir.resolve()
    stage = output / "CloudCompare-3DG-2.13.2-windows-x64"
    archive = Path(f"{stage}.zip")

    exe = build / "qCC/CloudCompare.exe"
    plugin_matches = list((build / "plugins/core/IO/qCoreIO").glob("*.dll"))
    if len(plugin_matches) != 1:
        raise RuntimeError(f"Expected one QCORE_IO_PLUGIN DLL, found: {plugin_matches}")
    if stage.exists():
        shutil.rmtree(stage)
    stage.mkdir(parents=True)
    copy(exe, stage / exe.name)
    copy(plugin_matches[0], stage / "plugins" / plugin_matches[0].name)

    qt_plugins = {
        "platforms": ("qwindows.dll",),
        "imageformats": ("qico.dll", "qjpeg.dll", "qsvg.dll"),
        "iconengines": ("qsvgicon.dll",),
        "mediaservice": ("dsengine.dll", "qtmedia_audioengine.dll"),
        "audio": ("qtaudio_windows.dll",),
        "bearer": ("qgenericbearer.dll",),
        "printsupport": ("windowsprintersupport.dll",),
        "styles": ("qwindowsvistastyle.dll",),
    }
    for subdir, names in qt_plugins.items():
        for name in names:
            copy(qt / "plugins" / subdir / name, stage / "plugins" / subdir / name)

    # Prefer the GCC 13 runtime used for this build over Qt's GCC 8 runtime.
    search_dirs = [
        build / "libs", build / "plugins",
        Path("/usr/lib/gcc/x86_64-w64-mingw32/13-posix"),
        Path("/usr/x86_64-w64-mingw32/lib"),
        qt / "bin",
    ]
    available: dict[str, Path] = {}
    for directory in reversed(search_dirs):
        if directory.is_dir():
            for path in directory.rglob("*.dll"):
                available[path.name.lower()] = path

    pending = list(stage.rglob("*.dll")) + [stage / exe.name]
    checked: set[Path] = set()
    missing: set[str] = set()
    while pending:
        binary = pending.pop()
        if binary in checked:
            continue
        checked.add(binary)
        for name in imports(binary, args.objdump):
            if name in SYSTEM_DLLS or name.startswith(("api-ms-win-", "ext-ms-win-")):
                continue
            dest = stage / name
            if not dest.exists():
                source = available.get(name)
                if source is None:
                    missing.add(name)
                    continue
                copy(source, dest)
            pending.append(dest)
    if missing:
        raise RuntimeError("Unresolved Windows DLL imports: " + ", ".join(sorted(missing)))

    shader_sources = [ROOT / "libs/CCFbo/shaders", ROOT / "qCC/shaders"]
    for source in shader_sources:
        shutil.copytree(source, stage / "shaders", dirs_exist_ok=True)
    (stage / "translations").mkdir(exist_ok=True)
    subprocess.run(
        [args.lrelease, str(ROOT / "qCC/translations/CloudCompare_zh.ts"),
         "-qm", str(stage / "translations/CloudCompare_zh.qm")],
        check=True,
    )
    copy(qt / "translations/qt_zh_CN.qm", stage / "translations/qt_zh_CN.qm")
    copy(ROOT / "license.txt", stage / "license.txt")
    copy(Path("/usr/share/common-licenses/GPL-2"), stage / "licenses/GPL-2.txt")
    copy(Path("/usr/share/common-licenses/GPL-3"), stage / "licenses/GPL-3.txt")
    copy(Path("/usr/share/common-licenses/LGPL-3"), stage / "licenses/LGPL-3.txt")
    copy(ROOT / ".deps/protobuf-3.21.12/LICENSE", stage / "licenses/PROTOBUF-LICENSE")
    copy(ROOT / "qCC/mission3dg/assets/x500/LICENSE", stage / "licenses/PX4-X500-LICENSE")
    (stage / "licenses/THIRD_PARTY_NOTICES.txt").write_text(
        "Qt 5.15.2 (dynamic libraries): https://www.qt.io/\n"
        "Protocol Buffers 3.21.12 (statically linked): https://github.com/protocolbuffers/protobuf\n"
        "CloudCompare 2.13.2: https://www.cloudcompare.org/\n"
        "See the accompanying license files and project source for terms and notices.\n",
        encoding="utf-8",
    )
    (stage / "qt.conf").write_text("[Paths]\nPlugins=plugins\n", encoding="utf-8")
    (stage / "Start-3DG.cmd").write_text(
        "@echo off\r\nset THREEDG=1\r\nstart \"\" \"%~dp0CloudCompare.exe\" %*\r\n",
        encoding="ascii",
    )
    (stage / "Start-3DG-Demo.cmd").write_text(
        "@echo off\r\nset THREEDG=1\r\nset THREEDG_DEMO=1\r\nstart \"\" \"%~dp0CloudCompare.exe\" %*\r\n",
        encoding="ascii",
    )
    (stage / "README-WINDOWS.txt").write_text(
        "CloudCompare 3DG 2.13.2 (Windows x64)\n"
        "解压后运行 Start-3DG.cmd；离线演示运行 Start-3DG-Demo.cmd。\n"
        "首次连接任务机前，在 3DG 配置中填写任务机地址。\n"
        "此包由 Linux 交叉编译，尚未在真实 Windows 桌面验证。\n"
        "三维界面需要 OpenGL 2.1 或更高版本的显卡驱动。\n"
        "视频播放依赖 Windows DirectShow 和本机可用的编解码器。\n",
        encoding="utf-8",
    )
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True, check=True
    ).stdout.strip()
    (stage / "BUILD_INFO.txt").write_text(
        f"Source revision: {revision}\n"
        "Build: Linux x86_64 -> Windows x86_64, MinGW GCC 13, Qt 5.15.2, Protobuf 3.21.12\n"
        "Source includes uncommitted local changes.\n",
        encoding="utf-8",
    )

    if archive.exists():
        archive.unlink()
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as zf:
        for path in sorted(stage.rglob("*")):
            if path.is_file():
                zf.write(path, path.relative_to(output))
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    (output / f"{archive.name}.sha256").write_text(f"{digest}  {archive.name}\n", encoding="ascii")
    print(f"Package: {archive}")
    print(f"SHA256: {digest}")
    print(f"PE files checked: {len(checked)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
