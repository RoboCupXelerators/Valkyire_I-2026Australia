#!/usr/bin/env python3
"""
OTA Updater - Over-The-Air 更新系统

功能：
1. 检查远程仓库是否有新版本
2. 下载并应用更新
3. 回滚机制

使用方法：
    from utils.ota_updater import OTAUpdater
    
    updater = OTAUpdater()
    if updater.check_update():
        updater.apply_update()
"""

import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# 尝试导入 requests，如果没有则使用 urllib
try:
    import requests
    HAS_REQUESTS = True
except ImportError:
    import urllib.request
    import urllib.error
    HAS_REQUESTS = False


class OTAUpdater:
    """OTA 更新器"""
    
    def __init__(self, config_path=None):
        self.project_root = self._get_project_root()
        self.config_path = config_path or (self.project_root / "ota_config.json")
        self.config = self._load_config()
        self.backup_dir = self.project_root / ".ota_backup"
        self.state_file = self.project_root / ".ota_state.json"
        
    def _get_project_root(self):
        """获取项目根目录"""
        # 从当前文件向上查找
        current = Path(__file__).resolve()
        while current.parent != current:
            if (current / "src").exists() or (current / "packages_config.json").exists():
                return current
            current = current.parent
        return Path.cwd()
    
    def _load_config(self):
        """加载配置"""
        default_config = {
            "repo_url": "https://github.com/your-org/Xelerators.git",
            "branch": "main",
            "check_interval": 3600,  # 检查间隔（秒）
            "auto_update": False,
            "backup_enabled": True,
            "packages_repo": "https://github.com/your-org/xelerators-packages.git"
        }
        
        if self.config_path.exists():
            with open(self.config_path, 'r') as f:
                user_config = json.load(f)
                default_config.update(user_config)
        
        return default_config
    
    def _save_state(self, state):
        """保存状态"""
        state["updated_at"] = datetime.now().isoformat()
        with open(self.state_file, 'w') as f:
            json.dump(state, f, indent=2)
    
    def _load_state(self):
        """加载状态"""
        if self.state_file.exists():
            with open(self.state_file, 'r') as f:
                return json.load(f)
        return {"last_check": None, "current_version": None, "update_history": []}
    
    def _http_get(self, url, timeout=30):
        """HTTP GET 请求"""
        if HAS_REQUESTS:
            resp = requests.get(url, timeout=timeout)
            resp.raise_for_status()
            return resp.text
        else:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return resp.read().decode('utf-8')
    
    def _run_command(self, cmd, cwd=None, check=True):
        """运行命令"""
        result = subprocess.run(
            cmd,
            cwd=cwd or self.project_root,
            capture_output=True,
            text=True
        )
        if check and result.returncode != 0:
            raise RuntimeError(f"Command failed: {cmd}\n{result.stderr}")
        return result
    
    def get_current_version(self):
        """获取当前版本"""
        try:
            result = self._run_command(["git", "rev-parse", "HEAD"], check=False)
            if result.returncode == 0:
                return result.stdout.strip()[:8]
        except Exception:
            pass
        
        # 从文件读取
        version_file = self.project_root / "VERSION"
        if version_file.exists():
            return version_file.read_text().strip()
        
        return "unknown"
    
    def get_remote_version(self):
        """获取远程最新版本"""
        repo_url = self.config["repo_url"]
        branch = self.config["branch"]
        
        try:
            # 使用 git ls-remote 获取最新 commit
            result = self._run_command(
                ["git", "ls-remote", repo_url, f"refs/heads/{branch}"],
                check=False
            )
            if result.returncode == 0 and result.stdout:
                return result.stdout.split()[0][:8]
        except Exception:
            pass
        
        # 尝试 GitHub API
        if "github.com" in repo_url:
            try:
                api_url = repo_url.replace(
                    "github.com", "api.github.com/repos"
                ).replace(".git", "") + f"/commits/{branch}"
                
                data = json.loads(self._http_get(api_url))
                return data["sha"][:8]
            except Exception:
                pass
        
        return None
    
    def check_update(self, force=False):
        """检查是否有更新
        
        Args:
            force: 强制检查，忽略检查间隔
            
        Returns:
            dict: {"has_update": bool, "current": str, "remote": str, "message": str}
        """
        state = self._load_state()
        
        # 检查间隔
        if not force:
            last_check = state.get("last_check")
            if last_check:
                last_time = datetime.fromisoformat(last_check)
                elapsed = (datetime.now() - last_time).total_seconds()
                if elapsed < self.config.get("check_interval", 3600):
                    return {
                        "has_update": False,
                        "current": self.get_current_version(),
                        "remote": None,
                        "message": f"Last check {int(elapsed)}s ago, skipping"
                    }
        
        # 更新检查时间
        state["last_check"] = datetime.now().isoformat()
        self._save_state(state)
        
        # 获取版本
        current = self.get_current_version()
        remote = self.get_remote_version()
        
        if not remote:
            return {
                "has_update": False,
                "current": current,
                "remote": None,
                "message": "Failed to get remote version"
            }
        
        has_update = current != remote
        
        return {
            "has_update": has_update,
            "current": current,
            "remote": remote,
            "message": "Update available" if has_update else "Already up to date"
        }
    
    def create_backup(self):
        """创建备份"""
        if not self.config.get("backup_enabled", True):
            return None
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = self.backup_dir / f"backup_{timestamp}"
        
        print(f"Creating backup at {backup_path}...")
        
        # 排除的目录
        exclude_dirs = {".git", "__pycache__", ".ota_backup", "packages", "venv", "node_modules"}
        
        # 创建备份目录
        backup_path.mkdir(parents=True, exist_ok=True)
        
        # 复制文件
        for item in self.project_root.iterdir():
            if item.name in exclude_dirs:
                continue
            
            dest = backup_path / item.name
            if item.is_dir():
                shutil.copytree(item, dest)
            else:
                shutil.copy2(item, dest)
        
        print(f"Backup created: {backup_path}")
        
        # 清理旧备份（保留最近 5 个）
        backups = sorted(self.backup_dir.glob("backup_*"))
        if len(backups) > 5:
            for old_backup in backups[:-5]:
                shutil.rmtree(old_backup)
                print(f"Removed old backup: {old_backup}")
        
        return backup_path
    
    def apply_update(self, backup=True):
        """应用更新
        
        Args:
            backup: 是否创建备份
            
        Returns:
            dict: {"success": bool, "message": str, "backup_path": str}
        """
        # 检查更新
        update_info = self.check_update(force=True)
        
        if not update_info["has_update"]:
            return {
                "success": True,
                "message": "Already up to date",
                "backup_path": None
            }
        
        # 创建备份
        backup_path = None
        if backup:
            try:
                backup_path = self.create_backup()
            except Exception as e:
                return {
                    "success": False,
                    "message": f"Backup failed: {e}",
                    "backup_path": None
                }
        
        try:
            print(f"Updating from {update_info['current']} to {update_info['remote']}...")
            
            # git pull
            result = self._run_command(
                ["git", "pull", "origin", self.config["branch"]],
                check=False
            )
            
            if result.returncode != 0:
                # 尝试 reset --hard
                print("Pull failed, trying reset...")
                self._run_command(["git", "fetch", "origin"])
                self._run_command([
                    "git", "reset", "--hard", 
                    f"origin/{self.config['branch']}"
                ])
            
            # 更新 packages
            self._update_packages()
            
            # 记录更新历史
            state = self._load_state()
            state["update_history"].append({
                "from": update_info["current"],
                "to": update_info["remote"],
                "timestamp": datetime.now().isoformat(),
                "backup": str(backup_path) if backup_path else None
            })
            self._save_state(state)
            
            print("Update completed!")
            
            return {
                "success": True,
                "message": f"Updated to {update_info['remote']}",
                "backup_path": str(backup_path) if backup_path else None
            }
            
        except Exception as e:
            # 回滚
            if backup_path:
                print(f"Update failed, rolling back from {backup_path}...")
                self._rollback(backup_path)
            
            return {
                "success": False,
                "message": f"Update failed: {e}",
                "backup_path": str(backup_path) if backup_path else None
            }
    
    def _update_packages(self):
        """更新 packages"""
        fetch_script = self.project_root / "scripts" / "fetch_packages.py"
        if fetch_script.exists():
            print("Updating packages...")
            self._run_command([sys.executable, str(fetch_script), "--upgrade"], check=False)
    
    def _rollback(self, backup_path):
        """回滚到备份"""
        backup_path = Path(backup_path)
        if not backup_path.exists():
            raise FileNotFoundError(f"Backup not found: {backup_path}")
        
        print(f"Rolling back from {backup_path}...")
        
        # 删除当前文件
        for item in self.project_root.iterdir():
            if item.name in {".git", ".ota_backup"}:
                continue
            
            if item.is_dir():
                shutil.rmtree(item)
            else:
                item.unlink()
        
        # 恢复备份
        for item in backup_path.iterdir():
            dest = self.project_root / item.name
            if item.is_dir():
                shutil.copytree(item, dest)
            else:
                shutil.copy2(item, dest)
        
        print("Rollback completed!")
    
    def get_update_history(self):
        """获取更新历史"""
        state = self._load_state()
        return state.get("update_history", [])
    
    def auto_update_check(self):
        """自动检查更新（用于后台运行）"""
        if not self.config.get("auto_update", False):
            return
        
        update_info = self.check_update()
        
        if update_info["has_update"]:
            print(f"Auto-updating to {update_info['remote']}...")
            result = self.apply_update(backup=True)
            print(f"Auto-update result: {result['message']}")


def main():
    """命令行入口"""
    import argparse
    
    parser = argparse.ArgumentParser(description="OTA Updater")
    parser.add_argument("--check", action="store_true", help="Check for updates")
    parser.add_argument("--update", action="store_true", help="Apply update")
    parser.add_argument("--rollback", type=str, help="Rollback to backup")
    parser.add_argument("--history", action="store_true", help="Show update history")
    parser.add_argument("--auto", action="store_true", help="Auto update (for cron)")
    
    args = parser.parse_args()
    
    updater = OTAUpdater()
    
    if args.check:
        result = updater.check_update(force=True)
        print(f"Current: {result['current']}")
        print(f"Remote: {result['remote']}")
        print(f"Status: {result['message']}")
        return
    
    if args.update:
        result = updater.apply_update()
        print(f"Result: {result['message']}")
        if result['backup_path']:
            print(f"Backup: {result['backup_path']}")
        return
    
    if args.rollback:
        updater._rollback(args.rollback)
        return
    
    if args.history:
        history = updater.get_update_history()
        for entry in history:
            print(f"{entry['timestamp']}: {entry['from']} -> {entry['to']}")
        return
    
    if args.auto:
        updater.auto_update_check()
        return
    
    # 默认显示状态
    result = updater.check_update(force=False)
    print(f"Current version: {result['current']}")
    print(f"Has update: {result['has_update']}")
    if result['has_update']:
        print(f"Remote version: {result['remote']}")


if __name__ == "__main__":
    main()