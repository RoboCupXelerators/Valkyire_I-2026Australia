# Kaiser 机器人面板接入与自动释放

## 自动释放

Kaiser 启动机器人面板时传入 `--idle-timeout-seconds 120`。最后一次浏览器请求完成后 120 秒内没有新访问，面板会正常退出，停止手动控制并释放相机和运行时资源。Kaiser 也会回收对应 SSH 转发，检查周期默认 10 秒。

- 多个浏览器共同续期；仍有请求在处理时不回收。
- Kaiser 的进程检查不会续期。`GET /api/panel/lifecycle` 也不会延长机器人面板的寿命。
- 重新打开或刷新面板首页时，Kaiser 会按配置重新启动面板和 SSH 转发。
- 自己直接运行 `python src/control_panel.py` 默认不会自动退出。只有启动时显式带超时参数的实例会退出，Kaiser 不会杀死其他 Python 程序。
- 更新前已经运行的面板保持原有行为，下一次由 Kaiser 启动后才启用超时。
- 120 秒是资源回收宽限期，不是运动指令看门狗的超时；既有的手动运动失联保护保持不变。

可在 Kaiser 的 `/etc/kaiser-field-ops/field-ops.yaml` 中按机器人设置：

```yaml
robot_panels:
  robots:
    robot4:
      # 保留该机器已有的 hostname、路径和端口字段
      idle_timeout_seconds: 120
```

`0` 表示不自动退出，也不按闲置时间回收隧道。未配置时默认为 120 秒。

## 其他机器人接入

以下以 `robot3` 为例，SSH 用户名、Python 路径和程序路径按实际安装调整。机器人名称必须已在 Kaiser 的 `modules` 中注册，主机名以机器人名称开头，例如 `robot3.local`。

1. 在机器人上更新 Software 并安装依赖，确认下面的帮助中包含 `--idle-timeout-seconds`：

   ```bash
   $HOME/Software/venv/bin/python $HOME/Software/src/control_panel.py --help
   ```

2. 在 Kaiser 输出面板专用的 SSH 公钥；已存在的密钥不会被替换：

   ```bash
   sudo /opt/kaiser-field-ops/.venv/bin/python /opt/kaiser-field-ops/scripts/configure_robot_panel.py bootstrap
   ```

   将输出的公钥加入机器人账户的 `$HOME/.ssh/authorized_keys`。不是复制私钥，也不是配置自己电脑到机器人的登录。

3. 在机器人本机确认主机密钥指纹：

   ```bash
   ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub -E sha256
   ```

   在 Kaiser 查看收到的指纹，并与上面的值比较：

   ```bash
   sudo /opt/kaiser-field-ops/.venv/bin/python /opt/kaiser-field-ops/scripts/configure_robot_panel.py fingerprint --host robot3.local
   ```

4. 在 Kaiser 写入接入配置，把 `SHA256:...` 替换为已核验的实际指纹：

   ```bash
   sudo /opt/kaiser-field-ops/.venv/bin/python /opt/kaiser-field-ops/scripts/configure_robot_panel.py enable \
     --robot robot3 --host robot3.local --ssh-user robot3 \
     --panel-path $HOME/Software/src/control_panel.py \
     --python $HOME/Software/venv/bin/python \
     --remote-port 8080 --local-port 19082 \
     --host-key-sha256 SHA256:... --idle-timeout-seconds 120
   ```

   脚本会先验证主机密钥、专用 SSH 登录和面板参数支持，再保存配置。它保留其他机器的配置和 known_hosts 条目，拒绝复用已分配的本地端口，并备份修改前的文件。不会启动机器人或修改蓝牙绑定。

5. 配好所有机器后，在 Kaiser 执行一次：

   ```bash
   sudo systemctl restart kaiser-field-ops
   ```

   随后长按对应机器人卡片打开面板。也可以给最后一次 `enable` 加上 `--restart`。

不同机器可以都使用远端 `8080`，但 Kaiser 上的 `local_port` 必须不同。例如 robot4 使用 `19081`，robot3 可使用 `19082`。

## 查看实例状态

机器人本机的只读状态接口：

```bash
curl http://127.0.0.1:8080/api/panel/lifecycle
```

返回的 `idle_timeout_seconds` 为 `120` 表示该实例已开启自动释放；为 `0` 表示普通常驻实例。旧版本没有这个接口。
