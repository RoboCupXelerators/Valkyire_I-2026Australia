#!/usr/bin/env python3
"""
Package Fetcher - 从 packages 仓库拉取大文件

使用方法:
    python scripts/fetch_packages.py              # 拉取所有包
    python scripts/fetch_packages.py --check      # 检查本地包状态
    python scripts/fetch_packages.py --upgrade    # 更新到最新版本

配置:
    在 packages_config.json 中设置:
    - packages_repo: packages 仓库的 URL 或本地路径
    - packages: 需要拉取的包列表
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import urllib.request
import urllib.error
from pathlib import Path

# 默认配置
DEFAULT_CONFIG = {
    "packages_repo": "https://github.com/your-org/xelerators-packages.git",
    "packages_branch": "main",
    "packages_dir": "packages",
    "packages": {
        "hailort-4.23.0-cp312-cp312-linux_aarch64.whl": {
            "size": 9932383,
            "sha256": "auto",  # auto = 从仓库计算
            "platform": ["linux_aarch64"],
            "description": "Hailo runtime wheel for Jetson"
        },
        "hailort_4.23.0_arm64.deb": {
            "size": 6877660,
            "sha256": "auto",
            "platform": ["linux_aarch64"],
            "description": "Hailo runtime deb for Jetson"
        },
        "hailort-pcie-driver_4.23.0_all.deb": {
            "size": 142680,
            "sha256": "auto",
            "platform": ["linux_aarch64"],
            "description": "Hailo PCIe driver"
        },
        "tegra234-p3768-0000+p3767-0005-nv-super.dtb": {
            "size": 249487,
            "sha256": "auto",
            "platform": ["linux_aarch64"],
            "description": "Jetson device tree blob"
        }
    },
    "version": "1.0.0"
}

CONFIG_FILE = "packages_config.json"
LOCK_FILE = "packages_lock.json"


def get_project_root():
    """获取项目根目录"""
    script_path = Path(__file__).resolve()
    return script_path.parent.parent


def load_config():
    """加载配置文件"""
    config_path = get_project_root() / CONFIG_FILE
    if config_path.exists():
        with open(config_path, 'r') as f:
            return json.load(f)
    return DEFAULT_CONFIG


def save_config(config):
    """保存配置文件"""
    config_path = get_project_root() / CONFIG_FILE
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)


def load_lock():
    """加载 lock 文件"""
    lock_path = get_project_root() / LOCK_FILE
    if lock_path.exists():
        with open(lock_path, 'r') as f:
            return json.load(f)
    return {"packages": {}, "version": None}


def save_lock(lock):
    """保存 lock 文件"""
    lock_path = get_project_root() / LOCK_FILE
    with open(lock_path, 'w') as f:
        json.dump(lock, f, indent=2)


def calculate_sha256(file_path):
    """计算文件的 SHA256"""
    sha256_hash = hashlib.sha256()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            sha256_hash.update(chunk)
    return sha256_hash.hexdigest()


def get_current_platform():
    """获取当前平台"""
    import platform
    system = platform.system().lower()
    machine = platform.machine().lower()
    
    if system == "linux":
        if machine in ["aarch64", "arm64"]:
            return "linux_aarch64"
        elif machine in ["x86_64", "amd64"]:
            return "linux_x86_64"
    elif system == "darwin":
        if machine in ["arm64", "aarch64"]:
            return "macos_arm64"
        elif machine in ["x86_64", "amd64"]:
            return "macos_x86_64"
    elif system == "windows":
        return "windows_x86_64"
    
    return f"{system}_{machine}"


def fetch_from_git_repo(config, package_name, dest_path):
    """从 git 仓库拉取包"""
    repo_url = config.get("packages_repo", DEFAULT_CONFIG["packages_repo"])
    branch = config.get("packages_branch", "main")
    
    # 使用 sparse checkout 只拉取需要的文件
    packages_dir = get_project_root() / config.get("packages_dir", "packages")
    
    # 创建临时目录
    temp_dir = packages_dir / ".temp"
    temp_dir.mkdir(parents=True, exist_ok=True)
    
    try:
        # 使用 git archive 拉取单个文件
        cmd = [
            "git", "archive", "--remote", repo_url,
            branch, f"packages/{package_name}"
        ]
        
        result = subprocess.run(
            cmd,
            cwd=temp_dir,
            capture_output=True,
            text=True
        )
        
        if result.returncode != 0:
            # 如果 git archive 失败，尝试 clone 整个仓库
            print(f"  git archive failed, trying clone...")
            clone_dir = temp_dir / "repo"
            if clone_dir.exists():
                shutil.rmtree(clone_dir)
            
            subprocess.run(
                ["git", "clone", "--depth", "1", "--branch", branch, repo_url, str(clone_dir)],
                check=True,
                capture_output=True
            )
            
            # 复制文件
            src_file = clone_dir / "packages" / package_name
            if src_file.exists():
                shutil.copy2(src_file, dest_path)
                return True
            return False
        
        # 解压 archive
        subprocess.run(
            ["tar", "-xf", "-", "-C", str(temp_dir)],
            input=result.stdout.encode(),
            check=True
        )
        
        # 移动文件
        extracted_file = temp_dir / package_name
        if extracted_file.exists():
            shutil.move(str(extracted_file), str(dest_path))
            return True
        
        return False
        
    finally:
        # 清理临时目录
        if temp_dir.exists():
            shutil.rmtree(temp_dir, ignore_errors=True)


def fetch_from_url(url, dest_path):
    """从 URL 拉取包"""
    print(f"  Downloading from {url}...")
    
    try:
        req = urllib.request.Request(url)
        with urllib.request.urlopen(req, timeout=60) as response:
            total_size = int(response.headers.get('Content-Length', 0))
            downloaded = 0
            
            with open(dest_path, 'wb') as f:
                while True:
                    chunk = response.read(8192)
                    if not chunk:
                        break
                    f.write(chunk)
                    downloaded += len(chunk)
                    if total_size > 0:
                        percent = (downloaded / total_size) * 100
                        print(f"    Progress: {percent:.1f}%")
        
        return True
    except urllib.error.URLError as e:
        print(f"  Download failed: {e}")
        return False


def check_package_status(config, package_name, packages_dir):
    """检查包的状态"""
    package_info = config["packages"].get(package_name)
    if not package_info:
        return "unknown"
    
    file_path = packages_dir / package_name
    
    if not file_path.exists():
        return "missing"
    
    # 检查大小
    actual_size = file_path.stat().st_size
    expected_size = package_info.get("size", 0)
    
    if actual_size != expected_size:
        return "size_mismatch"
    
    # 检查 SHA256（如果有）
    expected_sha256 = package_info.get("sha256")
    if expected_sha256 and expected_sha256 != "auto":
        actual_sha256 = calculate_sha256(file_path)
        if actual_sha256 != expected_sha256:
            return "hash_mismatch"
    
    return "ok"


def fetch_package(config, package_name, packages_dir, force=False):
    """拉取单个包"""
    package_info = config["packages"].get(package_name)
    if not package_info:
        print(f"  Unknown package: {package_name}")
        return False
    
    # 检查平台兼容性
    current_platform = get_current_platform()
    supported_platforms = package_info.get("platform", [])
    if supported_platforms and current_platform not in supported_platforms:
        print(f"  Skipping {package_name} (not for {current_platform})")
        return True
    
    dest_path = packages_dir / package_name
    
    # 检查现有状态
    if not force:
        status = check_package_status(config, package_name, packages_dir)
        if status == "ok":
            print(f"  {package_name}: already present and valid")
            return True
    
    print(f"  Fetching {package_name}...")
    
    # 尝试从 git 仓库拉取
    if fetch_from_git_repo(config, package_name, dest_path):
        print(f"  Fetched from git repo")
        return True
    
    # 尝试从 URL 拉取（如果有）
    url = package_info.get("url")
    if url and fetch_from_url(url, dest_path):
        print(f"  Downloaded from URL")
        return True
    
    print(f"  Failed to fetch {package_name}")
    return False


def fetch_all_packages(config, force=False):
    """拉取所有包"""
    packages_dir = get_project_root() / config.get("packages_dir", "packages")
    packages_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Fetching packages to {packages_dir}...")
    print(f"Platform: {get_current_platform()}")
    print()
    
    results = {}
    lock = load_lock()
    
    for package_name in config["packages"]:
        success = fetch_package(config, package_name, packages_dir, force)
        results[package_name] = success
        
        if success:
            file_path = packages_dir / package_name
            lock["packages"][package_name] = {
                "status": "ok",
                "size": file_path.stat().st_size if file_path.exists() else 0,
                "sha256": calculate_sha256(file_path) if file_path.exists() else None,
                "fetched_at": subprocess.run(
                    ["git", "log", "-1", "--format=%H", "--", str(file_path)],
                    cwd=get_project_root(),
                    capture_output=True,
                    text=True
                ).stdout.strip() if file_path.exists() else None
            }
    
    lock["version"] = config.get("version", "unknown")
    save_lock(lock)
    
    # 总结
    print()
    print("Summary:")
    success_count = sum(1 for v in results.values() if v)
    total_count = len(results)
    print(f"  {success_count}/{total_count} packages fetched successfully")
    
    return all(results.values())


def check_all_packages(config):
    """检查所有包的状态"""
    packages_dir = get_project_root() / config.get("packages_dir", "packages")
    
    print(f"Checking packages in {packages_dir}...")
    print(f"Platform: {get_current_platform()}")
    print()
    
    for package_name, package_info in config["packages"].items():
        status = check_package_status(config, package_name, packages_dir)
        platform_note = ""
        supported_platforms = package_info.get("platform", [])
        if supported_platforms:
            current_platform = get_current_platform()
            if current_platform not in supported_platforms:
                platform_note = f" (not needed for {current_platform})"
        
        status_icon = {
            "ok": "✓",
            "missing": "✗",
            "size_mismatch": "⚠",
            "hash_mismatch": "⚠",
            "unknown": "?"
        }.get(status, "?")
        
        print(f"  {status_icon} {package_name}: {status}{platform_note}")
    
    print()
    print(f"Total packages: {len(config['packages'])}")


def main():
    parser = argparse.ArgumentParser(description="Fetch packages from remote repository")
    parser.add_argument("--check", action="store_true", help="Check package status")
    parser.add_argument("--upgrade", action="store_true", help="Force upgrade all packages")
    parser.add_argument("--init", action="store_true", help="Create config file")
    parser.add_argument("--package", type=str, help="Fetch specific package")
    
    args = parser.parse_args()
    
    config = load_config()
    
    if args.init:
        save_config(config)
        print(f"Created {CONFIG_FILE}")
        return
    
    if args.check:
        check_all_packages(config)
        return
    
    force = args.upgrade
    
    if args.package:
        packages_dir = get_project_root() / config.get("packages_dir", "packages")
        packages_dir.mkdir(parents=True, exist_ok=True)
        success = fetch_package(config, args.package, packages_dir, force)
        sys.exit(0 if success else 1)
    
    success = fetch_all_packages(config, force)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()