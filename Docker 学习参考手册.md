# Docker 学习参考手册

本文面向具有 Linux 基础、希望从入门到进阶掌握 Docker 的开发者和运维人员。阅读顺序按照“概念 -> 原理 -> 使用 -> 工程实践 -> 排障”组织。文中的命令默认在 Bash 或兼容终端执行；涉及 Windows 时会特别说明。

> 版本提示：Docker CLI、Docker Engine、Docker Compose Plugin 和 Docker Desktop 的具体版本会影响个别参数和界面。执行 `docker version`、`docker compose version` 查看当前环境。

## 1. 核心概念

### 1.1 Docker 解决什么问题

Docker 使用镜像（Image）打包应用及其运行时依赖，再以容器（Container）的形式启动。它把“代码在谁的机器上能运行”的问题，转化为“使用同一个镜像和配置运行”的问题。

典型交付链路如下：

```text
Dockerfile -> docker build -> Image -> Registry -> docker pull -> Container
                                                |
                                      Volume / Network / Config
```

容器不是轻量虚拟机。容器中的进程与宿主机共享 Linux 内核，但通过 Namespace、Cgroups 等机制获得隔离的进程视图、网络视图、文件系统视图和资源配额。

### 1.2 镜像 Image

镜像是只读的分层文件系统和元数据集合，通常包含：

- 基础用户空间文件，如 Debian、Alpine 或 Ubuntu 的 root filesystem。
- 应用程序、依赖、配置和启动元数据。
- 每一层的内容摘要（digest）以及历史信息。

镜像名称通常写成：

```text
[registry[:port]/][namespace/]repository[:tag]
```

例如：

```text
nginx:1.27
registry.example.com/team/web:2026.09
ubuntu@sha256:<digest>
```

`tag` 是可读的移动标签，不一定永久指向同一内容；生产环境更适合使用经过发布流程管理的固定 tag 或 digest。

### 1.3 容器 Container

容器是镜像的一个运行实例，具有自己的可写层、进程、网络接口、环境变量和资源限制。多个容器可以由同一个镜像启动，但彼此的可写层默认隔离。

```bash
docker run -d --name web -p 8080:80 nginx:1.27
docker ps
docker stop web
docker rm web
```

容器的生命周期一般是：

```text
created -> running -> paused -> stopped/exited -> removed
```

容器停止通常意味着主进程（PID 1）退出，而不是容器数据自动消失。容器被 `docker rm` 删除后，其可写层会消失；需要持久化的数据应放到 Volume 或 Bind Mount。

### 1.4 仓库 Registry、Repository 与 Tag

- **Registry**：镜像仓库服务，例如 Docker Hub、GitHub Container Registry、Harbor 或云厂商 Registry。
- **Repository**：Registry 中的一组相关镜像名称，例如 `team/payment`。
- **Tag**：同一 Repository 下的版本或用途标记，例如 `v1.4.0`、`staging`。
- **Digest**：镜像内容的不可变摘要，例如 `sha256:...`。

常用流程：

```bash
docker login registry.example.com
docker tag myapp:local registry.example.com/team/myapp:1.0.0
docker push registry.example.com/team/myapp:1.0.0
docker pull registry.example.com/team/myapp:1.0.0
```

### 1.5 Dockerfile

Dockerfile 是构建镜像的声明式文本文件。它描述基础镜像、文件复制、依赖安装、默认用户以及容器启动方式。

```dockerfile
FROM python:3.12-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY . .
USER 10001
CMD ["python", "app.py"]
```

### 1.6 Volume、Bind Mount 与 tmpfs

docker exec -it web sh
docker exec web nginx -t

- **Bind Mount**：把宿主机指定路径挂载进容器，路径和权限由用户管理，适合开发源码映射和配置文件注入。
- **tmpfs mount**：数据存放在内存中，容器停止后消失，适合临时文件或敏感的短生命周期数据。

```bash
docker volume create db-data
docker run -d --name db -v db-data:/var/lib/postgresql/data postgres:16

docker run --rm -it --mount type=bind,src="$PWD",dst=/workspace alpine:3.20 sh
```

### 1.7 Network

Docker Network 为容器提供虚拟网络和服务发现。常见驱动包括 `bridge`、`host`、`none` 和 Swarm 使用的 `overlay`。用户自定义 bridge 网络通常会提供基于容器名称的 DNS 解析。

```bash
docker network create app-net
docker run -d --name redis --network app-net redis:7
# 同一网络中的应用可使用 redis:6379 访问 Redis
docker run --rm -it --network app-net alpine:3.20 sh
```

### 1.8 相关对象的关系

```text
Dockerfile --build--> Image --run--> Container
Image <-------------> Registry
Container --mount--> Volume / Bind Mount
Container --connect--> Network
Compose 文件 --------> 多个 Container、Network、Volume
```

## 2. 架构原理

### 2.1 Client-Server 架构

Docker 使用 Client-Server 模型：

- **Docker CLI**：用户执行的 `docker` 命令，是客户端。
- **Docker Engine API**：客户端与服务端通信的 REST API，可通过 Unix socket 或 TCP 暴露。
- **Docker Daemon（dockerd）**：服务端，负责镜像、容器、网络、Volume 等对象的管理和调度。
- **Registry**：提供镜像分发，不是 Docker Daemon 的替代品。

默认 Linux 上 CLI 通过 `/var/run/docker.sock` 连接本机 Daemon：

```bash
docker context ls
docker context show
docker info
```

远程连接应使用 TLS、SSH 或受控的 Docker Context。不要把未经认证的 Docker API 直接暴露到公网，因为拥有 Docker socket 的权限通常等价于拥有宿主机 root 级别的控制能力。

### 2.2 dockerd、containerd 与 runc

典型调用路径如下：

```text
docker CLI
   |
Docker Engine API
   |
dockerd
   |
containerd 负责镜像、容器生命周期、镜像传输和存储协作
   |
containerd-shim 管理具体容器进程
   |
runc 按 OCI Runtime Specification 创建并启动容器
   |
Linux kernel
```

职责可以这样理解：

- `dockerd`：面向 Docker 用户的高层编排与 API 服务。
- `containerd`：通用容器运行时管理器，处理镜像、快照、容器生命周期等。
- `runc`：低层 OCI runtime，配置 Namespace、Cgroups、root filesystem 并启动进程。
- OCI 标准：规定镜像格式和运行时接口，使 Docker 生态可以与其他工具协作。

Docker Desktop 还会在 macOS 和 Windows 上运行一个 Linux 虚拟机，Linux 容器实际运行在该虚拟机内，而不是直接运行在宿主机内核上。

### 2.3 Linux Namespace 隔离

Namespace 为进程提供隔离的系统视图，常见类型包括：

- `pid`：容器看到独立的进程树，容器内的首进程通常是 PID 1。
- `net`：独立的网络接口、路由表和端口空间。
- `mnt`：独立的挂载点视图。
- `uts`：独立的 hostname 和 domain name。
- `ipc`：隔离 System V IPC 和 POSIX message queue。
- `user`：映射容器内外的用户和组 ID，可用于 rootless 模式。
- `cgroup`：隔离进程看到的 Cgroups 层级视图。

Namespace 主要解决“看见什么”；它不负责限制 CPU、内存等资源。

### 2.4 Cgroups 资源控制

Cgroups 主要解决“能用多少”：

- CPU 配额、CPU 权重和 CPU 集合。
- 内存上限、交换分区使用和 OOM 行为。
- 块设备 I/O 权重或限速。
- PIDs 数量限制。

```bash
docker run -d --name limited \
  --memory=512m \
  --cpus=1.5 \
  --pids-limit=200 \
  nginx:1.27
docker stats limited
```

容器内进程仍可能看到宿主机的部分内核信息；隔离不是绝对安全边界。高安全场景应结合最小权限、rootless、seccomp、AppArmor/SELinux、只读文件系统和专用运行时等措施。

## 3. 安装与配置

### 3.1 Linux：Docker Engine

生产环境优先使用 Docker 官方仓库或发行版维护的受支持安装方式，避免混用多个来源的同名软件包。以 Debian/Ubuntu 为例，流程通常是：

1. 卸载旧的非官方包（例如发行版中的旧 `docker.io`），确认没有需要保留的本地数据。
2. 安装 `ca-certificates`、`curl`、GPG 工具等基础依赖。
3. 添加 Docker 官方仓库及其签名密钥。
4. 安装 `docker-ce`、`docker-ce-cli`、`containerd.io`、Buildx 和 Compose Plugin。
5. 启动并设置服务开机启动。
6. 用 `hello-world` 验证。

```bash
sudo systemctl enable --now docker
sudo systemctl status docker
docker run --rm hello-world
```

让普通用户使用 Docker：

```bash
sudo usermod -aG docker "$USER"
# 重新登录，或在当前会话执行：
newgrp docker
docker run --rm hello-world
```

`docker` 用户组权限很高。多用户服务器应评估 rootless Docker 或更严格的授权边界。

Rootless 模式通常通过发行版或 Docker 提供的安装脚本配置：

```bash
dockerd-rootless-setuptool.sh install
docker context ls
```

具体依赖（如 `newuidmap`、`newgidmap`、subuid/subgid 配置）以当前发行版文档为准。

### 3.2 Windows

Windows 开发者通常安装 Docker Desktop：

1. 启用硬件虚拟化和 WSL 2 或 Hyper-V。
2. 安装 Docker Desktop。
3. 选择 WSL 2 backend（适合多数 Linux 容器开发场景）。
4. 在 Docker Desktop 中启用需要使用的 WSL 发行版集成。
5. 在 PowerShell 或 WSL 中执行验证命令。

```powershell
docker version
docker run --rm hello-world
docker compose version
```

Windows Server 可根据版本和目标选择 Windows containers、Linux containers 或 Mirantis/containerd 等受支持方案。Windows 容器镜像必须与目标 Windows 内核版本兼容，不能简单地把 Linux 镜像当作 Windows 镜像运行。

### 3.3 macOS

macOS 上通常使用 Docker Desktop，它通过 Linux 虚拟机运行 Linux 容器：

1. 安装与 Apple Silicon 或 Intel 架构匹配的 Docker Desktop。
2. 启动 Docker Desktop，等待 Engine 就绪。
3. 根据需要配置 CPU、内存、磁盘镜像和文件共享。
4. 执行验证命令。

```bash
docker version
docker run --rm hello-world
docker context show
```

Apple Silicon 上优先使用支持 `linux/arm64` 的镜像；只有 `linux/amd64` 镜像时可以使用模拟，但通常会有性能代价：

```bash
docker run --platform linux/amd64 --rm image:tag
```

跨平台发布镜像：

```bash
docker buildx build --platform linux/amd64,linux/arm64 \
  -t registry.example.com/team/app:1.0.0 --push .
```

### 3.4 镜像加速与 daemon.json

Linux Docker Daemon 的默认配置文件一般是 `/etc/docker/daemon.json`。不同地区和组织可配置可信的 Registry mirror；镜像地址应使用组织批准的服务，不要盲目复制未知来源的地址。

```json
{
  "registry-mirrors": [
    "https://mirror.example.com"
  ],
  "log-driver": "local",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  },
  "live-restore": true,
  "default-address-pools": [
    {
      "base": "172.30.0.0/16",
      "size": 24
    }
  ]
}
```

修改后检查 JSON 并重启：

```bash
sudo dockerd --validate --config-file=/etc/docker/daemon.json
sudo systemctl restart docker
docker info
```

注意：`daemon.json` 是 Daemon 级配置，不能把 Compose 配置或 CLI 参数直接写进去。已有 Network 的地址不会自动重新分配，地址池调整前要评估现有网络冲突和业务影响。

## 4. 常用命令

### 4.1 命令结构与帮助

```bash
docker --help
docker <command> --help
docker version
docker info
docker system df
```

### 4.2 镜像管理

```bash
# 搜索、拉取、列出镜像
docker search nginx
docker pull nginx:1.27
docker image ls

# 查看元数据和历史
docker image inspect nginx:1.27
docker history nginx:1.27

# 构建、标记、推送
docker build -t myapp:dev .
docker tag myapp:dev registry.example.com/team/myapp:1.0.0
docker push registry.example.com/team/myapp:1.0.0

# 删除和清理未使用镜像
docker image rm myapp:dev
docker image prune
docker image prune -a
```

`prune -a` 可能删除本地没有容器引用的镜像，执行前确认是否需要离线使用。

### 4.3 容器生命周期

```bash
# 创建并后台运行
docker run -d --name web -p 127.0.0.1:8080:80 nginx:1.27

# 查看运行中/全部容器
docker ps
docker ps -a

# 查看状态和日志
docker inspect web
docker logs --tail=100 -f web

# 执行命令、进入 Shell
docker exec -it web sh

docker exec web nginx -t

# 停止、启动、重启、删除
docker stop web
docker start web
docker restart web
docker rm web

# 强制删除运行中的容器
docker rm -f web
```

容器名应体现服务用途；端口发布建议显式绑定监听地址。`-p 8080:80` 通常会监听所有宿主机地址，而 `-p 127.0.0.1:8080:80` 只允许本机访问。

### 4.4 环境变量、工作目录和用户

```bash
docker run --rm \
  --env APP_ENV=production \
  --env-file .env \
  --workdir /app \
  --user 10001:10001 \
  myapp:1.0
```

不要把密码、Token、私钥直接写入镜像层、Dockerfile 或 Shell 历史；使用运行时 Secret 管理方案。

### 4.5 网络命令

```bash
docker network ls
docker network create --driver bridge app-net
docker network inspect app-net
docker network connect app-net web
docker network disconnect app-net web
docker network rm app-net
docker network prune
```

端口发布是“宿主机 -> 容器”的转发，不等于容器已经具备服务发现能力。自定义网络中的容器应使用服务名或容器别名通信。

### 4.6 存储命令

```bash
docker volume ls
docker volume create app-data
docker volume inspect app-data
docker volume rm app-data
docker volume prune
```

备份 Volume 的一种方式：

```bash
docker run --rm \
  -v app-data:/source:ro \
  -v "$PWD":/backup \
  alpine:3.20 \
  tar czf /backup/app-data.tgz -C /source .
```

### 4.7 日志与诊断

```bash
docker logs --since=10m --timestamps web
docker events
docker top web
docker stats --no-stream
docker port web
docker diff web
```

`docker logs` 通常读取容器主进程的 stdout/stderr。应用如果把日志只写入容器内文件，Docker 默认日志驱动可能无法采集，需要配置日志驱动、Sidecar/Agent 或应用自身的日志输出策略。

### 4.8 资源限制

```bash
docker run -d --name worker \
  --cpus=2 \
  --memory=1g \
  --memory-swap=1g \
  --pids-limit=256 \
  --restart=unless-stopped \
  worker:1.0
```

常用选项：

- `--cpus`：CPU 配额。
- `--memory`：内存上限。
- `--memory-swap`：内存加 swap 的总上限；设为与 `--memory` 相同可限制 swap 使用。
- `--pids-limit`：进程数量上限。
- `--restart`：退出后的自动重启策略，如 `no`、`on-failure`、`unless-stopped`。
- `--read-only`：将容器 root filesystem 设为只读，必要时配合 `--tmpfs /tmp`。

## 5. Dockerfile 详解

### 5.1 常用指令

| 指令            | 作用                                | 示例                                                |
| --------------- | ----------------------------------- | --------------------------------------------------- |
| `FROM`        | 指定基础镜像                        | `FROM node:22-alpine`                             |
| `ARG`         | 构建时变量                          | `ARG VERSION=dev`                                 |
| `ENV`         | 镜像/运行时环境变量                 | `ENV NODE_ENV=production`                         |
| `WORKDIR`     | 设置工作目录                        | `WORKDIR /app`                                    |
| `COPY`        | 从构建上下文复制文件                | `COPY package*.json ./`                           |
| `ADD`         | 复制文件，额外支持归档和 URL 等行为 | 尽量优先`COPY`                                    |
| `RUN`         | 构建阶段执行命令并生成新层          | `RUN npm ci`                                      |
| `EXPOSE`      | 声明容器预期监听端口                | `EXPOSE 8080`                                     |
| `USER`        | 指定后续命令和默认进程用户          | `USER app`                                        |
| `VOLUME`      | 声明匿名 Volume                     | 需谨慎使用                                          |
| `CMD`         | 默认运行命令，可被覆盖              | `CMD ["node", "server.js"]`                       |
| `ENTRYPOINT`  | 固定容器入口                        | `ENTRYPOINT ["/usr/local/bin/app"]`               |
| `HEALTHCHECK` | 定义健康检查                        | `HEALTHCHECK CMD curl -f http://localhost/health` |
| `SHELL`       | 修改 Shell 解释器                   | Windows 场景较常见                                  |

### 5.2 CMD 与 ENTRYPOINT

推荐使用 exec form，避免 Shell form 带来的信号转发和参数解析问题：

```dockerfile
ENTRYPOINT ["/usr/local/bin/myapp"]
CMD ["--config", "/etc/myapp/config.yaml"]
```

此时：

```bash
docker run myapp:1.0                 # 使用默认参数
docker run myapp:1.0 --config /tmp/a.yaml  # 覆盖 CMD 参数
```

如果程序需要响应 `SIGTERM` 并优雅退出，确保它尽量直接作为 PID 1 运行，或使用正确的 init 进程（例如 `--init`）。

### 5.3 构建上下文与 .dockerignore

`docker build .` 中的 `.` 是构建上下文。Dockerfile 只能访问上下文内的文件；构建上下文过大会拖慢上传和构建，也可能把敏感文件带入构建流程。

```text
.git
.gitignore
node_modules
__pycache__
*.log
.env
.env.*
Dockerfile*
```

`.dockerignore` 不等于安全边界，真正的 Secret 不应放在构建上下文中。需要构建时 Secret 时，使用 BuildKit 的 Secret 机制，不要用 `ARG` 或 `ENV` 写入镜像历史。

### 5.4 多阶段构建

多阶段构建让编译环境和运行环境分离，最终镜像只复制运行所需产物：

```dockerfile
FROM golang:1.24 AS build
WORKDIR /src
COPY go.mod go.sum ./
RUN go mod download
COPY . .
RUN CGO_ENABLED=0 go build -trimpath -o /out/server ./cmd/server

FROM gcr.io/distroless/static-debian12:nonroot
COPY --from=build /out/server /server
USER nonroot:nonroot
EXPOSE 8080
ENTRYPOINT ["/server"]
```

构建：

```bash
docker buildx build --target build -t myapp:builder .
docker buildx build -t myapp:1.0 .
```

### 5.5 Dockerfile 最佳实践

1. 选择可信、足够小且仍受支持的基础镜像，并固定到明确版本。
2. 将变更少的依赖安装层放在变化频繁的源码复制之前。
3. 合并相关的安装和清理操作，避免缓存无效文件进入镜像层。
4. 使用 `.dockerignore` 缩小上下文。
5. 使用多阶段构建，最终镜像不包含编译器、包管理缓存和测试工具。
6. 尽量使用非 root 用户运行应用。
7. 使用 exec form 的 `CMD`/`ENTRYPOINT`，正确处理信号和退出码。
8. 不在 Dockerfile 中写入密码、私钥和访问 Token。
9. 为关键服务提供 `HEALTHCHECK`，但不要把复杂业务探活逻辑塞进镜像基础设施检查。
10. 使用 BuildKit、缓存挂载和镜像扫描工具，持续检查漏洞与许可证。

## 6. 数据与网络

### 6.1 Volume 与 Bind Mount 对比

| 维度     | Volume                       | Bind Mount                     |
| -------- | ---------------------------- | ------------------------------ |
| 存储位置 | Docker 管理                  | 用户指定宿主机路径             |
| 可移植性 | 较好，适合服务数据           | 依赖宿主机目录结构             |
| 权限控制 | 需处理容器用户与 Volume 权限 | 直接受宿主机权限影响           |
| 典型用途 | 数据库、上传文件、持久化队列 | 源码热更新、配置注入、开发调试 |
| 备份     | 通过 Volume 或临时容器导出   | 直接备份宿主机目录             |

只读挂载：

```bash
docker run --rm \
  --mount type=bind,src="$PWD/config",dst=/etc/myapp,readonly \
  myapp:1.0
```

### 6.2 容器之间互联

```bash
docker network create backend
docker run -d --name database --network backend \
  -e POSTGRES_PASSWORD=change-me postgres:16
docker run -d --name api --network backend \
  -e DATABASE_HOST=database myapi:1.0
```

在 `backend` 网络中，`api` 通过 `database:5432` 访问数据库。不要把数据库端口发布到宿主机，除非确实需要宿主机或外部客户端访问。

### 6.3 网络模式

- `bridge`：默认隔离网络，适合单机多容器。
- 自定义 `bridge`：支持更清晰的网络边界、名称解析和别名。
- `host`：容器直接使用宿主机网络命名空间，减少网络开销但牺牲隔离和端口独立性。
- `none`：禁用网络，适合完全不需要网络的任务。
- `overlay`：跨 Docker 节点网络，通常与 Swarm 等编排方案配合。

### 6.4 DNS、端口与健康检查

容器访问外部服务需要正确的 DNS、路由和代理配置。容器间访问通常使用内部端口，而外部用户访问使用发布后的宿主机端口。

```bash
docker run -d --name api \
  --network app-net \
  -p 127.0.0.1:8080:8080 \
  --health-cmd='wget -qO- http://127.0.0.1:8080/health || exit 1' \
  --health-interval=30s \
  --health-timeout=3s \
  --health-retries=3 \
  myapi:1.0
```

## 7. Docker Compose

### 7.1 Compose 的定位

Docker Compose 使用一个 YAML 文件声明多容器应用，包括服务、网络、Volume、环境变量、健康检查和依赖关系。现代 Compose 使用命令 `docker compose`，而不是旧的独立 `docker-compose`。

### 7.2 完整示例

```yaml
services:
  api:
    build:
      context: ./api
      target: runtime
    image: example/api:dev
    ports:
      - "127.0.0.1:8080:8080"
    environment:
      DATABASE_URL: postgres://app:change-me@db:5432/app
    depends_on:
      db:
        condition: service_healthy
    networks:
      - backend
    restart: unless-stopped

  db:
    image: postgres:16
    environment:
      POSTGRES_DB: app
      POSTGRES_USER: app
      POSTGRES_PASSWORD: change-me
    volumes:
      - db-data:/var/lib/postgresql/data
    healthcheck:
      test: ["CMD-SHELL", "pg_isready -U app -d app"]
      interval: 10s
      timeout: 5s
      retries: 5
    networks:
      - backend

volumes:
  db-data:

networks:
  backend:
    driver: bridge
```

生产环境不应把真实密码长期写在 YAML 中，可使用 `.env`、外部 Secret 管理系统或 Compose 支持的 secrets 机制。`depends_on` 控制启动顺序或健康条件，不代表应用具备重试能力；应用仍应实现连接重试和故障恢复。

### 7.3 YAML 与变量替换

```yaml
services:
  web:
    image: "${WEB_IMAGE:-nginx:1.27}"
    ports:
      - "${HTTP_PORT:-8080}:80"
```

常用字段：

- `services`：服务定义，每个服务通常对应一个容器集合。
- `image` / `build`：使用已有镜像或从 Dockerfile 构建。
- `command` / `entrypoint`：覆盖启动命令。
- `environment` / `env_file`：注入环境变量。
- `ports`：发布端口。
- `expose`：仅声明容器间可用端口，不发布到宿主机。
- `volumes`：声明挂载。
- `networks`：加入网络。
- `healthcheck`：定义健康状态。
- `depends_on`：声明依赖关系。
- `profiles`：按场景启用可选服务。

### 7.4 常用命令

```bash
# 校验并渲染最终配置
docker compose config

# 构建并后台启动
docker compose up -d --build

# 查看状态、日志、服务列表
docker compose ps
docker compose logs -f api
docker compose top

# 执行命令和进入容器
docker compose exec api sh

# 扩容无状态服务
docker compose up -d --scale api=3

# 停止并删除容器、网络
docker compose down

# 同时删除声明的 Volume（谨慎）
docker compose down -v
```

建议把开发、测试、生产差异拆成 Compose override 文件或 profiles，并在 CI 中执行 `docker compose config` 进行语法和变量检查。

## 8. 进阶主题

### 8.1 镜像分层与缓存

每条多数会修改文件系统的 Dockerfile 指令会产生镜像层。构建时 Docker 会根据指令、上下文文件和依赖关系复用缓存；某一层失效后，后续层通常也要重新构建。

不佳的顺序：

```dockerfile
COPY . .
RUN npm ci
```

更利于缓存的顺序：

```dockerfile
COPY package.json package-lock.json ./
RUN npm ci
COPY . .
```

BuildKit 缓存挂载示例：

```dockerfile
# syntax=docker/dockerfile:1
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --no-cache-dir -r requirements.txt
```

导出和使用构建缓存：

```bash
docker buildx build \
  --cache-from=type=local,src=.build-cache \
  --cache-to=type=local,dest=.build-cache-new,mode=max \
  -t myapp:dev .
```

### 8.2 BuildKit、buildx 与多平台

BuildKit 是现代 Docker 构建后端，提供并行构建、Secret/SSH mount、缓存导出、多平台构建等能力。`buildx` 是使用这些能力的 CLI 插件。

```bash
docker buildx ls
docker buildx create --name multiarch --use
docker buildx inspect --bootstrap
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -t registry.example.com/team/app:1.0.0 \
  --push .
```

多平台镜像发布后，客户端会根据自身平台选择匹配的 manifest。原生构建节点通常比 QEMU 模拟更快、更稳定。

### 8.3 私有 Registry

常见私有 Registry 方案包括原生 Distribution Registry、Harbor、云厂商 Container Registry 和企业内部 Registry。上线时至少考虑：

- TLS 证书和域名。
- 身份认证、团队权限和最小推送/拉取权限。
- 镜像不可变标签或发布策略。
- 镜像签名、SBOM 和漏洞扫描。
- 清理策略、保留策略和跨区域复制。
- 审计日志、备份与灾难恢复。

本地测试 Registry：

```bash
docker run -d --restart=always --name registry \
  -p 127.0.0.1:5000:5000 \
  registry:2

docker tag myapp:dev localhost:5000/myapp:dev
docker push localhost:5000/myapp:dev
```

生产环境不要把没有 TLS 和认证的 Registry 直接暴露给不可信网络。

### 8.4 安全实践

1. 采用最小权限：非 root 用户、只读 root filesystem、限制 Linux capabilities。
2. 删除不需要的 capability：

```bash
docker run --rm \
  --cap-drop=ALL \
  --security-opt=no-new-privileges:true \
  --read-only \
  --tmpfs /tmp \
  myapp:1.0
```

3. 使用默认 seccomp，并按需配置 AppArmor 或 SELinux。
4. 不使用 `--privileged`，除非已明确理解其影响并完成隔离评估。
5. 保护 Docker socket；不要把 `/var/run/docker.sock` 随意挂载给应用容器。
6. 扫描基础镜像和最终镜像，定期更新依赖并重建。
7. 使用 digest 或受控的不可变版本，避免生产环境依赖漂移的 `latest`。
8. 把 Secret 放到专门的 Secret 管理系统，不放进 Git、镜像层、环境变量日志或构建参数。
9. 限制容器 CPU、内存、PIDs 和文件系统写权限，降低资源耗尽风险。
10. 对容器运行时、宿主机内核、镜像来源和 CI 构建器同时做安全审计。

### 8.5 性能优化

- 减小构建上下文和最终镜像，减少网络传输、启动和扫描时间。
- 使用多阶段构建、BuildKit cache mount 和稳定的依赖层。
- 根据业务选择基础镜像；小镜像不应以缺少调试能力或兼容性为代价。
- 将高 I/O 数据放在合适的 Volume 和存储设备上，避免把数据库数据放在临时可写层。
- 在 macOS/Windows 上减少跨虚拟机文件共享，源码挂载目录应关注 I/O 性能。
- 为容器设置合理的 CPU、内存和日志轮转限制。
- 使用 `docker stats`、应用指标、宿主机指标和 tracing 判断瓶颈，而不是只看容器启动时间。
- 对多架构构建使用原生 builder 或缓存，避免所有构建都依赖模拟执行。

## 9. 与 Kubernetes 的关系

### 9.1 定位差异

| 维度     | Docker                             | Kubernetes                                  |
| -------- | ---------------------------------- | ------------------------------------------- |
| 主要对象 | Image、Container、Network、Volume  | Pod、Deployment、Service、ConfigMap、Secret |
| 主要范围 | 单机容器运行和开发交付             | 多节点集群编排和声明式运维                  |
| 调度     | 通常由用户或 Compose 管理          | Scheduler 根据资源和约束调度 Pod            |
| 故障恢复 | Restart policy、Compose 或人工处理 | 控制器持续将实际状态拉回期望状态            |
| 服务发现 | Docker Network DNS                 | Service、CoreDNS、Ingress/Gateway           |
| 存储     | Volume、Bind Mount                 | PersistentVolume、PVC、StorageClass         |
| 发布     | `docker run`、Compose、Swarm     | Deployment、StatefulSet、Job 等             |

### 9.2 协作方式

Dockerfile 构建的 OCI 镜像可以被 Kubernetes 使用。典型流程是：

```text
代码 -> Dockerfile -> buildx 构建镜像 -> Registry
      -> Kubernetes Deployment 引用镜像
      -> Pod 在节点上由 containerd 等运行时启动
```

现代 Kubernetes 节点通常直接使用 `containerd` 或其他符合 CRI 的运行时，不要求安装 Docker Engine。Docker 仍然常用于本地开发、镜像构建和 CI；Kubernetes 负责集群调度、服务发现、滚动更新、健康恢复和资源管理。

不要把 Compose 文件直接当作生产 Kubernetes 配置。可以使用 Kompose 等转换工具作为起点，但仍需根据 Kubernetes 的网络、存储、Secret、探针、资源请求/限制和安全策略重新设计。

## 10. 常见问题与排查

### 10.1 Docker 命令无法连接 Daemon

现象：`Cannot connect to the Docker daemon`。

```bash
docker context show
docker context ls
systemctl --user status docker  # rootless 场景
sudo systemctl status docker    # Linux system daemon
```

排查方向：确认 Docker Desktop 是否启动、Linux 服务是否运行、当前 Context 是否指向正确环境、`DOCKER_HOST` 是否设置错误，以及当前用户是否有 socket 权限。

### 10.2 容器一启动就退出

```bash
docker ps -a
docker logs <container>
docker inspect <container> --format '{{.State.Status}} {{.State.ExitCode}} {{.State.Error}}'
```

重点检查主进程是否完成后正常退出、启动命令和工作目录是否正确、配置文件和环境变量是否存在、程序是否因权限或架构不匹配失败。

### 10.3 端口访问失败

```bash
docker port <container>
docker inspect <container>
ss -lntp
curl -v http://127.0.0.1:8080
```

确认应用监听的是 `0.0.0.0` 而不是容器内的 `127.0.0.1`，确认端口映射方向为 `宿主机端口:容器端口`，并检查防火墙、云安全组和绑定地址。

### 10.4 容器之间无法通信

```bash
docker network inspect <network>
docker exec <client> getent hosts <service-name>
docker exec <client> sh -c 'nc -vz <service-name> <port>'
```

确认两个容器加入同一自定义网络，使用服务名而非 `localhost`。在容器中，`localhost` 指向当前容器自身。

### 10.5 镜像拉取失败或很慢

检查 Registry 地址、认证、DNS、代理、证书和镜像架构；执行：

```bash
docker info
docker pull --quiet image:tag
docker manifest inspect image:tag
```

若使用镜像加速，检查 Daemon 配置和加速器是否可信、可用，并确认目标镜像是否已经同步。

### 10.6 构建缓存没有命中

检查 Dockerfile 指令顺序、`.dockerignore` 是否有效、构建上下文是否频繁变化、基础镜像或依赖锁文件是否发生变化。使用：

```bash
docker build --progress=plain -t myapp:debug .
docker history myapp:debug
```

将依赖描述文件先复制并安装，再复制业务源码，通常能显著提高缓存复用率。

### 10.7 Volume 数据或权限异常

```bash
docker volume inspect <volume>
docker exec <container> id
docker exec <container> ls -la <mount-path>
```

检查挂载类型和目标路径是否正确、宿主机目录的 UID/GID、SELinux 标签、容器用户以及应用是否覆盖了挂载点内容。不要用删除 Volume 的方式“修复”问题，先完成备份。

### 10.8 容器内存不足或被 OOM Kill

```bash
docker stats --no-stream <container>
docker inspect <container> --format '{{.State.OOMKilled}} {{.State.ExitCode}}'
docker events --since 10m
```

区分应用自身崩溃和内核/运行时 OOM，检查内存限制、swap、应用堆配置和实际峰值；不要只盲目增加限制，应结合指标分析泄漏或突发流量。

### 10.9 Compose 配置不生效

```bash
docker compose config
docker compose ps
docker compose logs <service>
docker compose exec <service> env
```

确认当前目录、项目名、override 文件、`.env` 变量替换和实际生成的配置。修改 `environment` 后必要时重新创建容器：

```bash
docker compose up -d --force-recreate <service>
```

### 10.10 架构不匹配

现象可能是 `exec format error` 或镜像无法启动。检查：

```bash
uname -m
docker image inspect image:tag --format '{{.Architecture}}/{{.Os}}'
docker manifest inspect image:tag
```

构建并发布包含目标平台的多架构镜像，或明确使用 `--platform`；模拟运行适合验证，不一定适合生产性能。

### 10.11 磁盘空间耗尽

```bash
docker system df
docker builder du
docker system prune
```

分别检查未使用容器、镜像、Volume、BuildKit 缓存和日志文件。`docker system prune -a --volumes` 破坏性较强，生产主机执行前必须确认影响范围和备份策略。

## 11. 推荐学习路径

1. 使用 `docker run` 启动单个 Web 服务，掌握端口、环境变量、日志和生命周期。
2. 编写 Dockerfile，将一个真实项目构建成可复现镜像。
3. 使用 Volume、Network 和非 root 用户运行数据库加应用。
4. 使用 Compose 管理本地多服务开发环境和测试环境。
5. 学习 BuildKit、多阶段构建、镜像扫描和私有 Registry。
6. 结合 Namespace、Cgroups、OCI 和 containerd 理解运行时。
7. 将镜像交付到 Kubernetes，学习资源、探针、Service、Secret 和滚动发布。

最终应形成一套稳定的工程习惯：镜像可复现、配置与代码分离、数据独立持久化、网络边界清晰、权限最小化、资源有上限、日志可观测、镜像可扫描、发布可回滚。
