# Linux 自动驾驶 / 机器人算法研发参考手册

本文面向有 C++、Python 或 ROS 2 基础的感知、定位、规划、控制、SLAM、仿真和深度学习研发人员。默认环境为 Ubuntu LTS、Bash、systemd、ROS 2；命令中的 `<...>` 需要替换为实际值。凡是涉及系统级改动的命令，先确认目标主机和备份，再使用 `sudo`。

## 1. 基础概念与哲学

### 1.1 Kernel、发行版与 Ubuntu LTS

Linux kernel 负责进程调度、虚拟内存、文件系统、网络、设备驱动和系统调用；发行版把 kernel、GNU 工具链、包管理器、初始化系统、桌面和文档组合起来。Ubuntu LTS 的优势是 ROS 2、NVIDIA、Docker 和开发工具的社区支持稳定，但 ROS 2、CUDA、驱动和系统版本必须按官方兼容矩阵选择，不能只看“最新版”。

```bash
uname -a                         # kernel、架构
cat /etc/os-release              # 发行版版本
lsb_release -a                  # Ubuntu 版本（需 lsb-release）
arch                            # x86_64、aarch64 等
systemctl --version             # systemd 版本
```

### 1.2 FHS 与“一切皆文件”

FHS（Filesystem Hierarchy Standard）约定了目录职责：`/etc` 配置，`/var/log` 日志，`/var/lib` 持久化状态，`/usr/bin` 普通程序，`/usr/local` 管理员手工安装的软件，`/opt` 独立应用，`/home` 用户文件，`/tmp` 临时文件，`/dev` 设备，`/proc` 进程和内核伪文件系统，`/sys` 设备和内核对象。

“一切皆文件”并非所有对象都是普通磁盘文件，而是尽量使用统一的 `open/read/write/ioctl` 接口。传感器设备可能是 `/dev/video0`，进程信息来自 `/proc/<pid>`，网卡参数来自 `/sys/class/net`，这解释了为何 `cat`、`lsof` 和权限模型也能用于设备与运行时问题。

### 1.3 开源生态

GPL 等许可证关注衍生作品和分发义务，Apache-2.0、MIT 等规则不同；使用 ROS 2、PCL、OpenCV、CUDA、容器镜像时应保留许可证和 NOTICE。Issue、源码、包仓库、容器、CI 和社区文档共同组成工程生态。算法研发应记录系统、驱动、编译器、依赖和模型版本，保证实验可复现。

**高频命令速查**

```bash
uname -r; lsb_release -a
man <command>; <command> --help
which <command>; type <command>
printenv | sort; id; hostnamectl
```

## 2. Shell 与命令行

### 2.1 文件、文本、进程与网络

```bash
pwd; ls -lah; tree -L 2                 # 位置、详细列表、目录树
cd /path; mkdir -p data/{raw,processed} # 创建目录树
cp -a src dst; mv old new; rm -r path   # -a 保留属性；rm -r 谨慎
find . -type f -name '*.pcd' -size +1G   # 按类型、名称、大小查找
locate <name>; updatedb                 # 文件索引（可能不是实时）
head -n 20 file; tail -f app.log        # 查看首尾；持续跟踪日志
wc -l file; sort file | uniq -c        # 行数、排序统计
ps aux; pgrep -af <pattern>; ss -lntup  # 进程和监听端口
ip addr; ip route; ping -c 4 <host>     # 网络地址、路由、连通性
curl -v http://<host>:<port>/health    # 查看 HTTP 请求细节
```

### 2.2 管道、重定向与退出码

`|` 把前一个命令的标准输出交给下一个命令；`>` 覆盖文件，`>>` 追加，`2>` 重定向标准错误，`&>` 合并输出；`tee` 一边显示一边写文件。默认命令退出码 `0` 表示成功，非零表示失败。

```bash
ros2 topic list | sort
./run_eval.sh >logs/eval.log 2>&1
make 2>&1 | tee logs/build.log
cmd && echo 'success' || echo "failed: $?"
```

脚本中建议使用 `set -Eeuo pipefail`，但要理解 `set -e` 在条件判断、管道和子命令中的边界，不能把它当作完整错误处理。

### 2.3 通配符、正则与环境变量

Shell 通配符由 Shell 展开：`*` 任意字符串，`?` 单个字符，`[0-9]` 字符范围；正则由 `grep/sed/awk` 等程序解释，二者不是一回事。变量不加引号会产生空格分词和通配符再次展开。

```bash
for file in "${dataset_dir}"/*.pcd; do
  [[ -e "$file" ]] || continue
  echo "$file"
done
export ROS_DOMAIN_ID=42
export PATH="$HOME/.local/bin:$PATH"
printf '%s\n' "$HOME" "${CUDA_HOME:-unset}"
```

### 2.4 alias、历史和补全

```bash
alias ll='ls -lah'
history | grep ros2
Ctrl-r                 # 反向搜索历史
Tab                    # 命令、路径、参数补全
source ~/.bashrc       # 让配置在当前 Shell 生效
```

复杂 alias 应升级为函数或脚本，避免命令参数和错误处理变得不可见。

**高频命令速查**

```bash
ls -lah; find . -type f -name 'pattern'
grep -RIn --exclude-dir=.git 'pattern' .
cmd >out 2>err; cmd 2>&1 | tee run.log
printenv VAR; export VAR=value; source ~/.bashrc
```

## 3. 文件系统与权限

### 3.1 inode、链接和目录

文件名只是目录项到 inode 的映射；inode 保存类型、权限、所有者、时间和数据块位置。磁盘空间足够但 inode 用尽时仍会无法创建文件，可用 `df -i` 检查。

| 项目       | 软链接（symbolic link）   | 硬链接（hard link）          |
| ---------- | ------------------------- | ---------------------------- |
| 本质       | 保存目标路径的特殊文件    | 同一 inode 的另一个目录项    |
| 跨文件系统 | 可以                      | 不可以                       |
| 目标删除   | 链接失效                  | 数据仍可通过另一名称访问     |
| 目录       | 通常允许                  | 通常禁止，避免目录环         |
| 常见用途   | `current -> release-42` | 同文件系统内多名称、备份入口 |

```bash
ln -s /data/datasets/2026-09 current_dataset
ln source.bin source.hardlink
readlink -f current_dataset
ls -li source.bin source.hardlink
```

### 3.2 rwx、chmod、chown、umask、ACL

对普通文件，`r` 读内容、`w` 修改内容、`x` 执行；对目录，`r` 列目录、`w` 创建/删除目录项、`x` 进入和访问其子项。权限按 user/group/other 分组。

```bash
stat file; ls -l file
chmod u+x run.sh; chmod 640 config.yaml; chmod -R g+rwX shared/
chown user:group file; chgrp developers file
umask 027                         # 新文件通常 640，新目录通常 750
getfacl shared; setfacl -m g:developers:rwx shared
setfacl -m d:g:developers:rwx shared # 默认 ACL，影响新建对象
```

八进制 `4=r`、`2=w`、`1=x`；`755` 是 `rwxr-xr-x`，数据集通常不应使用 `777`。目录共享可用组权限和 setgid（`chmod g+s shared`），使新文件继承组；sticky bit（`chmod +t shared`）让用户只能删除自己拥有的文件，典型目录是 `/tmp`。

SUID/SGID 会让程序以文件所有者/组身份运行或让目录继承组，需谨慎审计：

```bash
find / -xdev -perm /6000 -type f 2>/dev/null
```

**高频命令速查**

```bash
ls -l; stat <path>; df -i
chmod 750 <dir>; chown -R user:group <dir>
getfacl <path>; setfacl -m u:user:rwx <path>
```

## 4. 用户与组管理

### 4.1 账户、组和认证文件

`/etc/passwd` 保存用户名、UID、GID、家目录和登录 Shell；密码哈希在 `/etc/shadow`，只有 root 可读。不要手工编辑，使用 `useradd/usermod/groupadd` 或 `adduser`。

```bash
sudo groupadd developers
sudo useradd -m -s /bin/bash -G developers, docker <user>
sudo passwd <user>
sudo usermod -aG developers <user> # -a 必须和 -G 一起，否则会覆盖原组
id <user>; getent passwd <user>; getent group developers
```

新组通常要重新登录或执行 `newgrp developers` 才能进入当前会话。

### 4.2 sudo、su 与 SSH 密钥

`sudo` 临时以授权身份执行单条命令；`su` 切换整个登录身份。日常开发不应直接使用 root。`sudoers` 应用 `visudo` 编辑，并尽量授权明确命令而非 `NOPASSWD: ALL`。

```bash
ssh-keygen -t ed25519 -C 'dev-machine'
ssh-copy-id <user>@<host>                 # 将公钥写入 authorized_keys
ssh -o IdentitiesOnly=yes <user>@<host>
chmod 700 ~/.ssh; chmod 600 ~/.ssh/id_ed25519 ~/.ssh/authorized_keys
```

服务端 `/etc/ssh/sshd_config` 可考虑 `PermitRootLogin no`、`PasswordAuthentication no`、限制 `AllowGroups`；修改后用 `sudo sshd -t` 检查，再 `sudo systemctl reload ssh`。必须保留一个已验证的 SSH 会话，防止锁死。

### 4.3 多用户共享算法开发机

每个用户使用独立家目录和虚拟环境；共享数据集使用专用组、setgid 和 ACL；模型缓存按项目划分；不要把私钥、云凭据或含个人信息的日志放在公共目录。GPU 分配可通过约定、容器、调度器或 `CUDA_VISIBLE_DEVICES` 管理。

**高频命令速查**

```bash
id; groups; who; w
sudo -l; sudo -u <user> <command>
ssh-keygen -t ed25519; ssh-copy-id user@host
```

## 5. 进程、作业与 systemd

进程由父进程创建，经历运行、睡眠、停止、僵尸等状态；PID 1 通常是 systemd。线程共享进程地址空间，ROS 2 executor 的线程调度会直接影响回调延迟。

```bash
ps -eo pid,ppid,user,stat,ni,pri,psr,pcpu,pmem,cmd --sort=-pcpu | head
top -H -p <pid>                 # -H 查看线程
htop; pgrep -af autoware
jobs -l; ./long_eval.sh &; fg %1; Ctrl-z; bg %1
nohup ./run_sim.sh >sim.log 2>&1 &
kill -TERM <pid>; kill -STOP <pid>; kill -CONT <pid>
```

| 命令              | 含义               | 使用原则               |
| ----------------- | ------------------ | ---------------------- |
| `kill -TERM`    | 请求进程清理后退出 | 默认首选，应用可捕获   |
| `kill -INT`     | 类似 Ctrl-C        | 交互式程序常用         |
| `kill -HUP`     | 常用于重载配置     | 依程序定义             |
| `kill -KILL/-9` | 内核强制终止       | 无法清理资源，最后手段 |

### 5.1 systemd

```bash
systemctl status my-node.service
sudo systemctl enable --now my-node.service
sudo systemctl restart my-node.service
journalctl -u my-node.service -f
systemctl list-units --failed
```

服务单元应定义 `User=`、`WorkingDirectory=`、`EnvironmentFile=`、`Restart=on-failure`、资源限制和日志策略；修改 `/etc/systemd/system/*.service` 后执行 `sudo systemctl daemon-reload`。

**高频命令速查**

```bash
ps aux; top -H -p PID; pgrep -af pattern
jobs -l; fg %1; bg %1; nohup cmd >log 2>&1 &
kill -TERM PID; systemctl status name
journalctl -u name --since '1 hour ago'
```

## 6. 软件包与依赖管理

APT（Debian/Ubuntu）解决二进制包、依赖、升级和签名；`dnf` 常见于 Fedora/RHEL。源码安装适用于发行版没有的版本或需要定制编译选项，但应放在 `/usr/local`、`/opt/<project>` 或容器内，记录 commit、编译器和选项，避免覆盖包管理器文件。

```bash
sudo apt update
apt search <name>; apt policy <name>
sudo apt install build-essential cmake git pkg-config
sudo apt remove <name>; sudo apt autoremove
sudo apt-mark hold <package>             # 暂缓自动升级
sudo dnf install <package>               # Fedora/RHEL
```

### 6.1 CUDA、cuDNN、TensorRT

版本关系至少包含：GPU 硬件计算能力、NVIDIA driver、CUDA toolkit、CUDA runtime、cuDNN、TensorRT、PyTorch/TensorFlow 编译版本和容器 runtime。`nvidia-smi` 显示 driver 支持的 CUDA 上限，不等于本机安装了对应 toolkit；`nvcc --version` 才反映编译器。优先使用 NVIDIA 官方容器或项目锁定的 CUDA 基础镜像。

```bash
nvidia-smi
nvcc --version
ldconfig -p | grep -E 'libcuda| libcudnn|libnvinfer'
python - <<'PY'
import torch
print(torch.__version__, torch.version.cuda, torch.cuda.is_available())
if torch.cuda.is_available(): print(torch.cuda.get_device_name(0))
PY
```

安装前查官方 compatibility matrix；不要混用 `/usr/local/cuda-*`、conda CUDA 和容器库。运行时出现 `libcudnn.so` 或 `libnvinfer.so` 找不到，先检查动态库路径和实际加载文件：`ldd <binary>`、`echo $LD_LIBRARY_PATH`。升级驱动前确认内核模块、远程登录和回滚方案。

### 6.2 ROS 2 与 Python 环境

```bash
source /opt/ros/<distro>/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
python3 -m venv .venv; source .venv/bin/activate
python -m pip install -U pip
conda create -n perception python=3.10
```

`rosdep` 根据 package.xml 和目标 ROS 发行版解析系统依赖；构建失败先区分缺包、CMake 查找失败、ABI 不兼容和 Python 包冲突。一个终端尽量只激活一个 Python 环境，记录 `pip freeze`/`conda env export`。CUDA 依赖常受 Python wheel 自带 runtime 影响，需确认实际加载版本。

**高频命令速查**

```bash
apt update; apt install <package>; apt policy <package>
nvidia-smi; nvcc --version
rosdep install --from-paths src --ignore-src -r -y
python -m venv .venv; source .venv/bin/activate
```

## 7. 磁盘、挂载与大数据 IO

`fdisk/parted` 操作分区，`mkfs` 创建文件系统，`mount` 挂载，`/etc/fstab` 配置启动挂载。执行分区和格式化前确认设备名，尤其是云主机、USB 盘和 NVMe。

```bash
lsblk -f; sudo fdisk -l
sudo parted /dev/<device> print
sudo mkfs.ext4 /dev/<partition>
sudo mkdir -p /mnt/dataset
sudo mount -o noatime /dev/<partition> /mnt/dataset
findmnt /mnt/dataset; blkid
sudoedit /etc/fstab             # 推荐使用 UUID=，先 mount -a 验证
```

ext4 通用稳定，XFS 适合大文件和高并发场景；LVM 可扩容和做快照：`pvcreate`、`vgcreate`、`lvcreate`、`lvextend -r`。空间排查：

```bash
df -hT; df -ih; du -xhd1 / | sort -h
sudo lsof +L1                       # 已删除但仍被进程占用的文件
sudo du -xhd1 /var /home 2>/dev/null | sort -h
```

点云和图像数据建议按日期/车辆/场景分层，使用 SSD/NVMe 做热数据、HDD/NAS 做归档，避免百万个小文件集中在单目录；采用 sharding、顺序读、压缩格式和索引。评估时用本地缓存，避免 NFS 上随机读取；使用 `fio` 测试而非凭感觉调优。`rsync --partial --info=progress2` 可断点同步，数据必须校验哈希和元数据。

**高频命令速查**

```bash
lsblk -f; findmnt; df -hT; df -ih
du -xhd1 <dir> | sort -h
sudo mount -a; sudo lsof +L1
rsync -aH --info=progress2 src/ host:/data/
```

## 8. 网络、配置与 ROS 2 DDS

TCP/IP 研发重点是链路、IP/子网、路由、端口、DNS、MTU 和防火墙。TCP 有连接、可靠和拥塞控制；UDP 延迟低但可靠性由应用或 DDS QoS 负责。

```bash
ip -br addr; ip route; ip neigh
ss -lntup; ping -c 4 <ip>; traceroute <host>
curl -v --connect-timeout 3 http://<host>:<port>
wget -c <url>
resolvectl status; resolvectl query <host>
sudo tcpdump -i <iface> -nn -s 0 port <port>
sudo ufw status verbose; sudo ufw allow from <subnet> to any port <port>
```

`127.0.0.1` 只表示本机，服务监听 `0.0.0.0` 才能被外部访问；开放端口前先明确来源网段。抓包时过滤方向、端口和协议，保存 `-w capture.pcap` 后用 Wireshark 分析。

### ROS 2 多机/DDS

多机 ROS 2 通信通常要求：主机互通、相同 `ROS_DOMAIN_ID`、相同 RMW/DDS 实现、可发现的网卡、兼容 QoS，且防火墙允许 DDS discovery 和 data ports。不同 DDS 实现对 multicast、网卡选择和 discovery server 的处理不同；跨网段或禁用 multicast 时，考虑 Fast DDS Discovery Server、Cyclone DDS peers 或 VPN。

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 daemon stop; ros2 daemon start
ros2 node list; ros2 topic list -t
ros2 topic info /topic --verbose
ip route get <peer-ip>
```

不要以“能 ping”证明 ROS 2 一定可用；`ping` 只验证 ICMP。可先用最小 talker/listener 验证域、RMW、网卡和 QoS，再接入 Autoware。

**高频命令速查**

```bash
ip -br addr; ip route; ss -lntup
ping -c 4 host; curl -v URL
sudo tcpdump -i any -nn port PORT
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION
ros2 topic info TOPIC --verbose
```

## 9. grep、sed、awk 与批量处理

```bash
grep -RInE 'ERROR|FATAL|segmentation fault' logs/
grep -vE '^#|^$' config.yaml
sed -n '1,80p' file; sed -E 's/[[:space:]]+$//' file
sed -i.bak -E 's/old_value/new_value/g' config/*.yaml
awk '{print $1, $3}' metrics.txt
awk -F, 'NR>1 && $4>0.1 {sum+=$4; n++} END {print sum/n}' results.csv
```

典型日志处理：按日期筛选、抽取节点名和延迟、统计错误频次；典型数据处理：读取 CSV 列、筛选异常帧、生成汇总。`grep` 找行，`sed` 做流式替换/选行，`awk` 按字段计算。复杂 JSON/YAML 使用 `jq`、Python 或专用解析器，不要用正则替代结构化解析。

```bash
journalctl -u perception --since today --no-pager \
  | grep -E 'WARN|ERROR' | awk '{print $5}' | sort | uniq -c | sort -nr
find dataset -name '*.pcd' -print0 | xargs -0 -n1 basename > manifest.txt
```

**高频命令速查**

```bash
grep -RInE 'pattern1|pattern2' path/
sed -n 'start,endp' file
awk -F, '{print $1}' data.csv
sort file | uniq -c | sort -nr
```

## 10. Shell 脚本与训练/评测自动化

```bash
#!/usr/bin/env bash
set -Eeuo pipefail
trap 'echo "failed at ${BASH_SOURCE[0]}:${LINENO}: ${BASH_COMMAND}" >&2' ERR

usage() { echo "usage: $0 --data DIR --out DIR [--gpu ID]"; }
data_dir=''; out_dir=''; gpu='0'
while [[ $# -gt 0 ]]; do
  case "$1" in
    --data) data_dir="$2"; shift 2;;
    --out) out_dir="$2"; shift 2;;
    --gpu) gpu="$2"; shift 2;;
    -h|--help) usage; exit 0;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2;;
  esac
done
[[ -d "$data_dir" ]] || { echo "missing data" >&2; exit 1; }
mkdir -p "$out_dir"
CUDA_VISIBLE_DEVICES="$gpu" python evaluate.py --data "$data_dir" --out "$out_dir"
```

变量用引号，数组用 `("${items[@]}")`，不要把路径拼成未加引号的字符串；函数用 `local`；`$?` 要立即保存；临时文件使用 `mktemp` 并在 `trap` 中清理。训练脚本应记录 git commit、参数、环境、GPU、开始时间和退出码，并支持断点、重试和幂等输出。

```bash
for seed in 0 1 2; do
  python train.py --seed "$seed" --output "runs/seed-$seed" \
    2>&1 | tee "runs/seed-$seed.log"
done
bash -n run.sh; shellcheck run.sh
bash -x run.sh --data data --out results   # 逐行调试
```

**高频命令速查**

```bash
bash -n script.sh; shellcheck script.sh
set -x; set +x
printf '%q\n' "$value"
wait; trap 'cleanup' EXIT
```

## 11. 性能监控、绑核与实时性

先测量再优化：确认是 CPU 饱和、内存不足/抖动、磁盘等待、网络丢包、GPU 利用率低还是锁竞争。一次只改变一个变量，并保留基线。

```bash
top; free -h; vmstat 1
sudo iostat -xz 1; sar -n DEV 1
pidstat -p <pid> -t -u -r -d 1
watch -n 1 nvidia-smi
nvtop; gpustat -cpu -p
```

CPU 绑核是把线程限制到逻辑 CPU，影响调度和 cache；GPU 选择是把 CUDA 设备映射给进程，两者不是一回事：

| 项目 | CPU affinity                            | GPU device selection                    |
| ---- | --------------------------------------- | --------------------------------------- |
| 工具 | `taskset`、`numactl`、cgroup cpuset | `CUDA_VISIBLE_DEVICES`、容器 GPU 参数 |
| 作用 | 限制 CPU 执行位置、NUMA 内存            | 限制进程可见的 CUDA 设备                |
| 示例 | `taskset -c 2-5 cmd`                  | `CUDA_VISIBLE_DEVICES=1 cmd`          |
| 注意 | 线程数、IRQ、NUMA、实时优先级           | 映射后程序内编号可能从 0 重新开始       |

```bash
taskset -cp 2-5 <pid>
numactl --cpunodebind=0 --membind=0 ./node
chrt -p <pid>; taskset -pc <pid>
CUDA_VISIBLE_DEVICES=1 python infer.py
```

PREEMPT_RT 将内核更多路径变为可抢占，配合线程优先级、CPU 隔离、IRQ 亲和性和锁策略降低延迟抖动；它不是“自动实时”，还需用 `cyclictest` 等测量，并评估驱动、GPU 和网络设备限制。不要随意提高 `SCHED_FIFO` 优先级，否则可能饿死系统线程。

**高频命令速查**

```bash
top -H -p PID; vmstat 1; iostat -xz 1
pidstat -p PID -t 1; free -h
nvidia-smi; nvtop; gpustat
taskset -cp PID; numactl --hardware
```

## 12. 日志、调试与故障排查

```bash
journalctl -b -p warning..alert
journalctl -u <service> --since '30 min ago' -f
dmesg -T | tail -n 100
sudo lsof -p <pid>; sudo lsof -i :<port>
strace -f -tt -T -p <pid>
strace -f -o trace.log ./program
ltrace ./program                     # 库调用，信息可能较少
```

段错误先保留 core dump 和二进制符号：`ulimit -c unlimited`，使用 `gdb ./app core`，执行 `bt full`、`thread apply all bt`。内存问题可用 Valgrind 或 AddressSanitizer：

```bash
gdb -q ./app
(gdb) run --config config.yaml
(gdb) bt full
(gdb) thread apply all bt
valgrind --leak-check=full --track-origins=yes ./app
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' ..
```

死锁要同时看所有线程栈、锁顺序和 core；TSan 更适合数据竞争，但不能与 ASan 随意混用。`strace` 适合“卡在哪里”：文件不存在、权限拒绝、连接超时、futex 等；高开销场景使用采样工具。

通用流程：复现并记录版本和命令 -> 缩小到最小案例 -> 看应用日志/退出码 -> 看 systemd、kernel、资源和依赖 -> 用 `lsof/strace/gdb` 验证假设 -> 修复后重复原场景并保存证据。

**高频命令速查**

```bash
journalctl -b; dmesg -T; lsof -p PID; lsof -i :PORT
strace -f -tt -T -o trace.log cmd
gdb ./app core; bt full
valgrind --leak-check=full ./app
```

## 13. 安全与权限加固

最小权限原则：用户、服务、容器只获得所需文件、设备、端口和系统调用。SSH 禁止 root 远程登录，优先密钥认证，限制来源网段；防火墙采用默认拒绝、按需放行；敏感配置使用权限 600 和 secret manager，不写入 git、镜像层和公开日志。

```bash
sudo ufw default deny incoming
sudo ufw default allow outgoing
sudo ufw allow from <trusted-subnet> to any port 22 proto tcp
sudo ufw enable
sudo aa-status                       # AppArmor 状态
getenforce                           # SELinux（若安装）
find / -xdev -type f -perm -002 -ls 2>/dev/null
```

AppArmor 以路径策略为主，SELinux 以标签和强制访问控制为主；Ubuntu 默认常见 AppArmor，RHEL 系常见 SELinux。常见攻击面包括暴露 SSH/Docker socket、过宽 sudo、未更新 kernel/驱动、可写的 SUID 程序、第三方脚本、含凭据的日志和公共 ROS 2 网络。安全策略不能破坏研发可观测性，应记录例外和期限。

**高频命令速查**

```bash
sudo ufw status numbered; sudo ufw delete <num>
sudo sshd -t; sudo systemctl reload ssh
sudo aa-status; getenforce
find / -xdev -perm /6000 -type f 2>/dev/null
```

## 14. 开发工具、Git、容器与嵌入式

### 14.1 编译工具链

```bash
sudo apt install gcc g++ make cmake ninja-build gdb pkg-config
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j"$(nproc)"
cmake --build build --target test
```

`gcc/g++` 是编译器，`make/ninja` 执行构建图，CMake 生成构建文件；`Debug` 便于调试，`Release` 优化，`RelWithDebInfo` 常适合算法验证。记录编译器 ABI、C++ 标准、依赖和 `CMAKE_PREFIX_PATH`。

### 14.2 Git 与大文件

```bash
git status; git diff; git log --oneline --decorate -10
git switch -c feature/name
git add path; git commit -m '...'
git lfs install; git lfs track '*.bag' '*.onnx' '*.pcd'
git lfs ls-files
```

Git 管理代码和小型配置，Git LFS 管理模型、bag 和数据清单，不建议把完整数据集塞进仓库。提交中写清实验目的；用 tag、commit 和 manifest 绑定模型与结果。

### 14.3 Docker 与 GPU 容器

```bash
docker build -t project:dev .
docker run --rm -it --gpus all --net=host \
  -v "$PWD":/workspace -v /data:/data project:dev bash
docker compose up -d; docker logs -f <container>
docker system df
```

GPU 容器需要宿主机 NVIDIA driver、Docker runtime 和 `nvidia-container-toolkit`；容器内 CUDA runtime 不能替代宿主机 driver。验证：

```bash
docker run --rm --gpus all nvidia/cuda:<tag>-base-ubuntu<ver> nvidia-smi
```

使用固定 image tag/digest、非 root 用户、只读挂载和最小 capability。ROS 2 容器常需 `--net=host`、设备权限、X11/Wayland 或显卡映射，但这些会降低隔离，按需要开启。

### 14.4 Remote、交叉编译与嵌入式

VS Code Remote-SSH 在目标机运行编译、语言服务器和调试器，本地只负责界面；目标机需 SSH、兼容 libc、足够磁盘和与工具链匹配的源码。交叉编译要区分 build/host/target，准备 sysroot、交叉编译器、依赖和目标架构，不能把主机的 `/usr/include` 随意混入。

```bash
ssh user@target
file build/app
readelf -h build/app; ldd build/app
# 示例：交叉编译器名称依平台而定
<aarch64-toolchain>-g++ --sysroot=<sysroot> main.cpp -o app
```

**高频命令速查**

```bash
cmake -S . -B build -G Ninja
cmake --build build -j$(nproc)
git status; git log --oneline -10; git lfs ls-files
docker run --rm --gpus all image nvidia-smi
file binary; readelf -h binary; ldd binary
```

## 15. 自动驾驶 / 机器人算法研发专项

### 15.1 ROS 2 工作空间

```bash
mkdir -p ~/ros_ws/src; cd ~/ros_ws
vcs import src < repositories.repos
source /opt/ros/<distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 pkg list; ros2 node list; ros2 topic list -t
ros2 topic echo /topic --once
ros2 launch <package> <file>.launch.xml
ros2 bag record -a -o run-001
ros2 bag info run-001; ros2 bag play run-001 --clock
rviz2
```

`--symlink-install` 便于 Python 和资源文件快速迭代；C++ 头文件或 ABI 改变后不要只依赖增量构建。按包构建可缩短反馈：`colcon build --packages-select <pkg>`；用 `--packages-up-to` 构建依赖闭包。launch 参数、namespace、remap、QoS 和 `/clock` 是回放/仿真的常见控制点。

### 15.2 深度学习环境

确认四层事实：`nvidia-smi` 看 driver，`nvcc` 看 toolkit，Python 框架打印编译 CUDA，实际模型运行看 `torch.cuda.is_available()`/设备名。PyTorch 和 TensorFlow 的 wheel/conda 包可能携带部分 runtime，多版本共存最好使用独立 venv/conda 或容器，不要频繁覆盖系统 CUDA。

```bash
python -m venv .venv-torch; source .venv-torch/bin/activate
python -m pip install torch torchvision
python -c 'import torch; print(torch.cuda.is_available(), torch.version.cuda)'
python -m pip freeze > requirements.lock.txt
```

### 15.3 数据集、模型与共享存储

数据目录应区分 `raw/`、`derived/`、`cache/`、`manifests/`、`runs/`；原始数据只读，处理产物记录输入 manifest、代码 commit、参数、随机种子和容器 digest。模型保存权重、配置、类别表、预处理版本和评测指标。共享存储需处理并发写入、锁、权限、带宽、断点同步和校验。

### 15.4 仿真与可视化

Gazebo、Isaac Sim、CARLA 常受 GPU driver、OpenGL/Vulkan、CUDA、Python、ROS bridge、显存和共享内存影响。先运行官方最小示例，再接入算法；分离渲染进程与无头计算，使用 `--headless` 或虚拟显示时确认实际图形后端。

```bash
glxinfo -B 2>/dev/null | head
vulkaninfo --summary 2>/dev/null | head
nvidia-smi --query-gpu=name,driver_version,memory.used --format=csv
```

调试与可视化：`matplotlib` 适合曲线和时间序列，`Open3D` 适合点云，Foxglove 适合 ROS 2 topic/bag，PlotJuggler 适合实时/回放信号。可视化异常先判断数据是否真的错误，再判断坐标系、时间戳、QoS 和渲染问题。

**高频命令速查**

```bash
source /opt/ros/<distro>/setup.bash; source install/setup.bash
colcon build --symlink-install --packages-select <pkg>
ros2 node list; ros2 topic list -t; ros2 topic info <topic> --verbose
ros2 bag record -a -o <bag>; ros2 bag play <bag> --clock
rviz2; nvidia-smi
```

## 16. 进阶主题

### 16.1 sysctl、cgroups、namespace

`sysctl` 读写内核参数；修改前理解范围、持久化方式和回滚。cgroups 限制 CPU、内存、IO、进程数，namespace 隔离 PID、网络、挂载、用户等视图，Docker 以此构建容器。

```bash
sysctl net.core.rmem_max
sudo sysctl -w net.core.rmem_max=...
sysctl -a | grep -E 'vm.swappiness|net.core'
systemd-cgls; systemd-cgtop
cat /proc/<pid>/cgroup; lsns -p <pid>
```

DDS 大消息、共享内存或高吞吐采集可能需要调整 socket buffer、`memlock` 或 `/dev/shm`，但应由测量驱动，不能盲目增大。

### 16.2 cron 与 at

```bash
crontab -e
# 每天 02:30 清理过期缓存
30 2 * * * /home/user/bin/cleanup.sh >>/home/user/log/cleanup.log 2>&1
at now + 10 minutes
atq; atrm <job-id>
```

cron 环境变量少、工作目录不同，应使用绝对路径、显式 `PATH` 和锁（如 `flock`），避免训练任务重复启动。systemd timer 更适合有日志、依赖和失败重启要求的任务。

### 16.3 perf 与共享内存/零拷贝

```bash
perf stat -d ./app
sudo perf top -p <pid>
perf record -g ./app; perf report
```

共享内存减少大点云/图像跨进程复制，但必须定义生命周期、所有权、同步、版本和崩溃恢复策略。ROS 2 intra-process communication、loaned message、DDS shared memory transport 是否真正零拷贝取决于消息类型、RMW、序列化和内存分配器，不能只看 API 名称。

```bash
ls -lh /dev/shm
ipcs -m; ipcrm -m <shmid>       # 清理前确认没有活跃使用者
```

### 16.4 NTP/PTP 与多传感器时间

多传感器融合需要区分设备时间、主机时间、ROS 时间和仿真 `/clock`。NTP 通常达到毫秒级，PTP（IEEE 1588）在支持硬件时间戳和正确网络配置时可达到更高精度。先统一时钟源，再测偏差、漂移和时间戳链路。

```bash
timedatectl; chronyc tracking; chronyc sources -v
pmc -u -b 0 'GET TIME_STATUS_NP'       # linuxptp 环境
sudo ptp4l -i <iface> -m
sudo phc2sys -s <iface> -c CLOCK_REALTIME -m
```

NTP/PTP 同步不等于传感器数据已经使用正确时间；还要检查驱动时间戳来源、时区、单调时钟与 wall clock 转换、bag 回放 `/clock` 以及传感器间外参/时间偏移标定。

**高频命令速查**

```bash
sysctl -a | grep <key>; systemd-cgtop; lsns -p PID
crontab -e; atq; perf stat -d cmd
ls -lh /dev/shm; ipcs -m
timedatectl; chronyc tracking; pmc -u -b 0 'GET TIME_STATUS_NP'
```

# 常见故障与排查清单

| 症状                          | 优先检查                                                              | 常见原因与动作                                                                                                                       |
| ----------------------------- | --------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| GPU 掉卡、`nvidia-smi` 失败 | `dmesg -T`、`nvidia-smi`、`lsmod`、PCIe 日志                    | 驱动/内核不匹配、过热、供电、PCIe 错误、容器 runtime；先保留日志和温度，确认宿主机问题还是容器问题，再重载/重启，不要先反复安装 CUDA |
| CUDA 程序找不到库             | `ldd`、`ldconfig -p`、`nvcc`、框架版本                          | `LD_LIBRARY_PATH` 指向错误、driver/runtime/toolkit 混用；使用锁定容器或清理库搜索顺序                                              |
| PyTorch 显示无 GPU            | `nvidia-smi`、`torch.version.cuda`、`torch.cuda.is_available()` | wheel/driver 不兼容、`CUDA_VISIBLE_DEVICES` 为空、容器没 `--gpus all`                                                            |
| ROS 2 多机发现失败            | `ping`、`ip route`、`ROS_DOMAIN_ID`、RMW、`tcpdump`           | 不同 domain/RMW、网卡选择、multicast/防火墙、跨网段 discovery；先用最小 talker/listener，再检查 QoS                                  |
| topic 可见但收不到数据        | `ros2 topic info --verbose`                                         | QoS 不兼容、publisher 已退出、namespace/remap 错误、时间/仿真时钟不对                                                                |
| 节点频繁崩溃                  | `journalctl`、core、`gdb bt full`、ASan                           | 空指针、越界、ABI/插件版本不匹配；保留带符号构建和最小复现                                                                           |
| 算法延迟突然升高              | `top -H`、`pidstat`、`iostat`、`nvidia-smi`、bag 时间戳       | CPU 抢占、swap、IO 等待、GPU 显存/同步、锁竞争、日志阻塞；先建立端到端时间线                                                         |
| 内存持续增长                  | RSS、`valgrind`、ASan、`/proc/PID/smaps_rollup`                   | 容器/消息缓存未释放、循环引用、线程栈、mmap/文件句柄；区分泄漏和缓存增长                                                             |
| 磁盘满但`du` 对不上         | `df -h`、`df -i`、`lsof +L1`                                    | 删除文件仍被进程打开、inode 耗尽、挂载点遮蔽；重启/释放前确认服务影响                                                                |
| bag/点云读写慢                | `iostat -xz`、`fio`、`df -hT`、网络吞吐                         | 小文件过多、NFS 随机 IO、压缩/解压 CPU、缓存不足；分片、顺序读、本地热缓存                                                           |
| 时间不同步、融合抖动          | `timedatectl`、`chronyc`、`ptp4l`、原始 timestamp               | NTP 未同步、PTP 网卡/主从错误、设备时间与 host time 混用、仿真`/clock` 未启用                                                      |
| SSH 登录失败                  | `ssh -vvv`、服务端 journal、权限                                    | `authorized_keys` 权限、用户名/密钥、sshd 配置、防火墙、Fail2ban；保留现有会话再改配置                                             |
| Docker GPU/图形不可用         | `docker info`、`docker run --gpus all ... nvidia-smi`             | toolkit/runtime 未安装、driver 在宿主机异常、设备/X11 权限；先验证最小 CUDA 容器                                                     |
| colcon 构建失败               | `rosdep`、CMake 输出、环境变量                                      | 未 source ROS、依赖缺失、overlay 顺序、旧 build/install、ABI；清理目标包后重建，不要盲目全量删除                                     |

## 推荐的日常检查顺序

```bash
# 1. 环境身份和版本
hostname; uname -r; lsb_release -a; git status
# 2. 资源
free -h; df -hT; nvidia-smi; uptime
# 3. 进程、端口和日志
ps aux --sort=-%cpu | head; ss -lntup; journalctl -b -p warning..alert
# 4. ROS 2 图和时间
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION
ros2 node list; ros2 topic list -t; timedatectl
# 5. 保存证据
script -q "logs/diagnosis-$(date +%F-%H%M%S).txt"
```

排障时记录命令、时间、主机、commit、容器 digest、驱动、内核、ROS 发行版、复现步骤和修复前后指标。可复现、可观测、可回滚，是算法研发环境长期稳定的核心能力。
