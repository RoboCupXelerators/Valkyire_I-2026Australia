"""OTA 客户端 - 模型与驱动管理

用于管理 AI 模型文件 (HEF/PT) 和驱动文件 (DTB/DEB) 的远程更新。
"""

from __future__ import annotations

import os
import json
import urllib.request
import urllib.error
import hashlib
import shutil
import subprocess
from pathlib import Path
from datetime import datetime
from typing import Optional, Dict, List
from dataclasses import dataclass

# 添加项目路径
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from ReasonData import logger, Settings
except ImportError:
    # 如果导入失败，使用简单替代
    import logging
    logger = logging.getLogger("OTA")
    logger.setLevel(logging.INFO)
    
    class _SimpleSettings:
        def __getattr__(self, name):
            return None
    Settings = _SimpleSettings()


@dataclass
class OTAUpdate:
    """OTA 更新项"""
    category: str
    current_version: str
    available_version: str
    files: Dict[str, dict]


class OTAClient:
    """OTA 客户端 - 管理模型和驱动的远程更新
    
    支持的文件类别:
    - model_hailo: Hailo HEF 模型文件
    - model_jetson: Jetson YOLO 模型 (PT/ENGINE)
    - driver_dtb: 设备树文件 (DTB)
    - driver_hailo: Hailo 驱动包 (DEB)
    """
    
    # 文件类别到本地路径的映射
    CATEGORY_PATHS = {
        "model_hailo": "YOLO_Models/models",
        "model_jetson": "YOLO_Models/models",
        "driver_dtb": "/boot",
        "driver_hailo": ".",
    }
    
    # 需要重启系统的类别
    REBOOT_REQUIRED = ["driver_dtb", "driver_hailo"]
    
    def __init__(self, config_path: Optional[str] = None, server: Optional[str] = None):
        self.logger = logger
        
        # 加载配置
        self.config = self._load_config(config_path)
        ota_cfg = self.config.get("OTA", {})
        
        # 支持命令行覆盖配置
        self.enabled = ota_cfg.get("Enabled", True)
        self.server = server or ota_cfg.get("Server", "")
        self.check_interval = ota_cfg.get("CheckInterval", 300)
        self.auto_install = ota_cfg.get("AutoInstall", False)
        self.categories = ota_cfg.get("Categories", list(self.CATEGORY_PATHS.keys()))
        
        # 本地版本记录
        self.version_file = Path(__file__).parent.parent / "ReasonData" / "data" / "ota_versions.json"
        self.local_versions = self._load_local_versions()
        
        # 项目根目录
        self.project_root = Path(__file__).parent.parent.parent
    
    def _load_config(self, config_path: Optional[str]) -> dict:
        """加载配置文件"""
        if config_path:
            path = Path(config_path)
        else:
            path = Path(__file__).parent.parent / "ReasonData" / "data" / "config.json"
        
        if path.exists():
            with open(path, 'r', encoding='utf-8') as f:
                return json.load(f)
        return {}
    
    def _load_local_versions(self) -> dict:
        """加载本地版本记录"""
        if self.version_file.exists():
            with open(self.version_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        return {}
    
    def _save_local_versions(self):
        """保存本地版本记录"""
        self.version_file.parent.mkdir(parents=True, exist_ok=True)
        with open(self.version_file, 'w', encoding='utf-8') as f:
            json.dump(self.local_versions, f, indent=2)
    
    def get_current_versions(self) -> dict:
        """获取当前已安装的版本"""
        return {
            cat: self.local_versions.get(cat, {}).get("version", "0.0.0")
            for cat in self.categories
        }
    
    def check_updates(self) -> List[OTAUpdate]:
        """检查所有类别的更新
        
        Returns:
            List[OTAUpdate]: 可用的更新列表
        """
        if not self.enabled:
            self.logger.info("OTA is disabled")
            return []
        
        if not self.server:
            self.logger.warning("OTA server not configured")
            return []
        
        updates = []
        
        try:
            url = f"{self.server}/api/check_all"
            data = json.dumps(self.get_current_versions()).encode()
            req = urllib.request.Request(
                url, 
                data=data, 
                headers={'Content-Type': 'application/json'},
                method='POST'
            )
            
            with urllib.request.urlopen(req, timeout=10) as response:
                result = json.loads(response.read().decode())
            
            for item in result.get("updates_available", []):
                updates.append(OTAUpdate(
                    category=item["category"],
                    current_version=item["current_version"],
                    available_version=item["available_version"],
                    files=item.get("files", {})
                ))
            
            self.logger.info(f"OTA check complete: {len(updates)} updates available")
            
        except urllib.error.URLError as e:
            self.logger.error(f"OTA check failed (network error): {e}")
        except json.JSONDecodeError as e:
            self.logger.error(f"OTA check failed (invalid response): {e}")
        except Exception as e:
            self.logger.error(f"OTA check failed: {e}")
        
        return updates
    
    def download_file(self, category: str, filename: str, checksum: str) -> Optional[Path]:
        """下载单个文件
        
        Args:
            category: 文件类别
            filename: 文件名
            checksum: SHA256 校验和
            
        Returns:
            Path: 下载后的文件路径，失败返回 None
        """
        url = f"{self.server}/api/download/{category}/{filename}"
        
        # 确定本地路径
        if category in ["model_hailo", "model_jetson"]:
            local_path = self.project_root / "YOLO_Models" / "models" / filename
        elif category == "driver_dtb":
            local_path = Path("/boot") / filename
        elif category == "driver_hailo":
            local_path = self.project_root / filename
        else:
            local_path = Path(filename)
        
        temp_path = Path(f"/tmp/ota_{filename}")
        
        try:
            self.logger.info(f"Downloading {filename} from {url}...")
            
            with urllib.request.urlopen(url, timeout=120) as response:
                with open(temp_path, 'wb') as f:
                    # 分块下载，支持大文件
                    while True:
                        chunk = response.read(8192)
                        if not chunk:
                            break
                        f.write(chunk)
            
            # 验证校验和
            with open(temp_path, 'rb') as f:
                downloaded_checksum = hashlib.sha256(f.read()).hexdigest()
            
            if downloaded_checksum != checksum:
                self.logger.error(f"Checksum mismatch for {filename}")
                self.logger.error(f"Expected: {checksum}")
                self.logger.error(f"Got: {downloaded_checksum}")
                temp_path.unlink()
                return None
            
            # 备份原文件（如果存在）
            if local_path.exists():
                backup_path = local_path.with_suffix(local_path.suffix + ".bak")
                self.logger.info(f"Backing up existing file to {backup_path}")
                shutil.copy(local_path, backup_path)
            
            # 移动到目标位置
            local_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.move(str(temp_path), str(local_path))
            
            self.logger.info(f"Downloaded and installed: {local_path}")
            return local_path
            
        except urllib.error.URLError as e:
            self.logger.error(f"Download failed (network error): {e}")
        except Exception as e:
            self.logger.error(f"Download failed: {e}")
        
        # 清理临时文件
        if temp_path.exists():
            temp_path.unlink()
        return None
    
    def install_model(self, category: str, files: Dict[str, dict]) -> bool:
        """安装模型文件
        
        Args:
            category: model_hailo 或 model_jetson
            files: 文件信息字典
            
        Returns:
            bool: 是否成功
        """
        success = True
        
        for filename, info in files.items():
            checksum = info.get("checksum", "")
            if not checksum:
                self.logger.warning(f"No checksum for {filename}, skipping")
                continue
            
            result = self.download_file(category, filename, checksum)
            if result is None:
                success = False
        
        return success
    
    def install_driver_dtb(self, files: Dict[str, dict]) -> bool:
        """安装 DTB 驱动
        
        注意: DTB 更新通常需要系统重启才能生效
        
        Args:
            files: DTB 文件信息
            
        Returns:
            bool: 是否成功
        """
        for filename, info in files.items():
            if not filename.endswith('.dtb'):
                continue
            
            checksum = info.get("checksum", "")
            result = self.download_file("driver_dtb", filename, checksum)
            if result is None:
                return False
        
        self.logger.warning("DTB installed. System reboot required for changes to take effect.")
        return True
    
    def install_driver_hailo(self, files: Dict[str, dict]) -> bool:
        """安装 Hailo 驱动
        
        使用 dpkg 安装 .deb 包
        
        Args:
            files: DEB 文件信息
            
        Returns:
            bool: 是否成功
        """
        for filename, info in files.items():
            if not filename.endswith('.deb'):
                continue
            
            checksum = info.get("checksum", "")
            result = self.download_file("driver_hailo", filename, checksum)
            if result is None:
                return False
            
            # 安装 deb 包
            try:
                self.logger.info(f"Installing {filename} with dpkg...")
                proc = subprocess.run(
                    ["sudo", "dpkg", "-i", str(result)],
                    capture_output=True,
                    text=True
                )
                
                if proc.returncode != 0:
                    self.logger.error(f"dpkg install failed: {proc.stderr}")
                    # 尝试修复依赖
                    self.logger.info("Attempting to fix dependencies...")
                    subprocess.run(["sudo", "apt-get", "install", "-f", "-y"], check=True)
                    # 再次尝试安装
                    subprocess.run(["sudo", "dpkg", "-i", str(result)], check=True)
                
                self.logger.info(f"Successfully installed {filename}")
                
            except subprocess.CalledProcessError as e:
                self.logger.error(f"Failed to install {filename}: {e}")
                return False
        
        self.logger.warning("Hailo driver installed. System reboot may be required.")
        return True
    
    def install_update(self, update: OTAUpdate) -> bool:
        """安装更新
        
        Args:
            update: OTA 更新项
            
        Returns:
            bool: 是否成功
        """
        self.logger.info(f"Installing update for {update.category}...")
        self.logger.info(f"Version: {update.current_version} -> {update.available_version}")
        
        if update.category in ["model_hailo", "model_jetson"]:
            success = self.install_model(update.category, update.files)
        elif update.category == "driver_dtb":
            success = self.install_driver_dtb(update.files)
        elif update.category == "driver_hailo":
            success = self.install_driver_hailo(update.files)
        else:
            self.logger.warning(f"Unknown category: {update.category}")
            return False
        
        if success:
            # 更新本地版本记录
            self.local_versions[update.category] = {
                "version": update.available_version,
                "installed_at": datetime.now().isoformat(),
                "files": list(update.files.keys())
            }
            self._save_local_versions()
            self.logger.info(f"Update installed: {update.category} -> {update.available_version}")
            
            # 检查是否需要重启
            if update.category in self.REBOOT_REQUIRED:
                self.logger.warning(f"Category {update.category} requires system reboot")
        
        return success
    
    def rollback(self, category: str) -> bool:
        """回滚到备份版本
        
        Args:
            category: 要回滚的类别
            
        Returns:
            bool: 是否成功
        """
        if category in ["model_hailo", "model_jetson"]:
            # 查找备份文件
            model_dir = self.project_root / "YOLO_Models" / "models"
            backup_files = list(model_dir.glob("*.hef.bak")) + list(model_dir.glob("*.pt.bak"))
            
            for backup in backup_files:
                original = backup.with_suffix("")  # 移除 .bak
                if original.exists():
                    self.logger.info(f"Restoring {original} from {backup}")
                    shutil.copy(backup, original)
            
            # 更新版本记录
            if category in self.local_versions:
                prev_version = self.local_versions.get(category, {}).get("previous_version", "unknown")
                self.local_versions[category]["version"] = prev_version
                self.local_versions[category]["rolled_back_at"] = datetime.now().isoformat()
                self._save_local_versions()
            
            return True
        
        self.logger.warning(f"Rollback not implemented for {category}")
        return False
    
    def run_update_cycle(self) -> bool:
        """运行完整的更新检查和安装流程
        
        Returns:
            bool: 是否成功完成（没有更新或有更新并安装成功）
        """
        updates = self.check_updates()
        
        if not updates:
            self.logger.info("No updates available")
            return True
        
        for update in updates:
            if self.auto_install:
                success = self.install_update(update)
                if not success:
                    return False
            else:
                self.logger.info(
                    f"Update available: {update.category} "
                    f"({update.current_version} -> {update.available_version})"
                )
                self.logger.info(f"Files: {list(update.files.keys())}")
        
        return True


def main():
    """命令行入口"""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="OTA Client - Model & Driver Management for Xelerators",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Categories:
  model_hailo    Hailo HEF model files
  model_jetson   Jetson YOLO models (PT/ENGINE)
  driver_dtb     Device tree files (DTB)
  driver_hailo   Hailo driver packages (DEB)

Examples:
  # Check for updates
  python ota_client.py --check
  
  # Show current versions
  python ota_client.py --status
  
  # Install all updates
  python ota_client.py --install
  
  # Install specific category
  python ota_client.py --install --category model_hailo
  
  # Specify server
  python ota_client.py --check --server http://192.0.2.100:8080
        """
    )
    
    parser.add_argument("--check", action="store_true", help="Check for updates")
    parser.add_argument("--install", action="store_true", help="Install available updates")
    parser.add_argument("--status", action="store_true", help="Show current versions")
    parser.add_argument("--rollback", action="store_true", help="Rollback to backup versions")
    parser.add_argument("--category", type=str, help="Specific category to update")
    parser.add_argument("--server", type=str, help="OTA server URL (overrides config)")
    parser.add_argument("--auto", action="store_true", help="Auto install without confirmation")
    
    args = parser.parse_args()
    
    client = OTAClient(server=args.server)
    
    if args.status:
        print("\n=== Current Installed Versions ===")
        versions = client.get_current_versions()
        for cat, ver in versions.items():
            files = client.local_versions.get(cat, {}).get("files", [])
            print(f"  {cat}: v{ver}")
            if files:
                print(f"    Files: {files}")
        print()
    
    elif args.check:
        print("\n=== Checking for Updates ===")
        print(f"Server: {client.server}")
        print()
        updates = client.check_updates()
        if updates:
            print("Updates available:")
            for u in updates:
                print(f"  [{u.category}]")
                print(f"    Current: v{u.current_version}")
                print(f"    Available: v{u.available_version}")
                print(f"    Files: {list(u.files.keys())}")
                print()
        else:
            print("No updates available.\n")
    
    elif args.install:
        print("\n=== Installing Updates ===")
        updates = client.check_updates()
        
        if not updates:
            print("No updates available to install.\n")
            return
        
        for update in updates:
            if args.category and update.category != args.category:
                continue
            
            print(f"\nUpdating {update.category}...")
            print(f"  Version: {update.current_version} -> {update.available_version}")
            print(f"  Files: {list(update.files.keys())}")
            
            if not args.auto:
                response = input("Proceed? [y/N]: ").strip().lower()
                if response != 'y':
                    print("Skipped.")
                    continue
            
            success = client.install_update(update)
            if success:
                print(f"  ✓ Successfully installed")
            else:
                print(f"  ✗ Installation failed")
    
    elif args.rollback:
        print("\n=== Rolling Back ===")
        if args.category:
            success = client.rollback(args.category)
            if success:
                print(f"  ✓ Rolled back {args.category}")
            else:
                print(f"  ✗ Rollback failed")
        else:
            for cat in client.categories:
                success = client.rollback(cat)
                if success:
                    print(f"  ✓ Rolled back {cat}")
    
    else:
        parser.print_help()


if __name__ == "__main__":
    main()