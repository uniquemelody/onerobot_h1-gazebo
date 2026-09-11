# A1 Gazebo 发布与授权维护指南

本文件仅面向已经完成本地查看和验证、并获组织明确授权的维护者。开始前请先完成 [README 的一次性 setup 和本地 4-case smoke](README.md)。所有命令都从仓库根目录执行，并且必须已有该本地 smoke 生成的 `dist/gazebo-fuel`。本页从发布专属的缩略图、授权、manifest、客户端交接、API / 原始 ZIP 和严格缓存安全门开始；它不提供日常 GUI 启动步骤。

## 缩略图：生成、人工检查并批准

渲染和批准是两个不同动作。先只生成候选：

~~~bash
set -euo pipefail
candidate_parent="$(mktemp -d)"
candidate_root="$candidate_parent/a1-thumbnail-candidates"
bash gazebo/scripts/render_thumbnails.sh render --models dist/gazebo-fuel --candidates "$candidate_root"
find "$candidate_root" -maxdepth 1 -type f -printf '%f\n' | sort
sha256sum "$candidate_root"/*.png "$candidate_root/render-manifest.json"
~~~

现在必须 review all three candidate PNG files：逐张确认机械臂完整、没有裁切、
方向正确、光照清楚、不是空白图；同时保存上面的 SHA-256。若任何一张不合格，
不要批准。

确认看到的就是该候选目录中的同一批字节后，再执行：

~~~bash
set -euo pipefail
bash gazebo/scripts/render_thumbnails.sh approve --candidates "$candidate_root"
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.package --output dist/gazebo-fuel
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.validate dist/gazebo-fuel
~~~

approve does not rerender；它只验证并原子复制已经人工看过的候选字节。重新
导出后，每个 Fuel 模型必须恰好有 thumbnails/0.png。

## 发布前：硬性授权门

做到这里仍然只是本地发布候选。真正发布需要同时具备：

- 由组织授权维护者使用的 Gazebo Fuel account 和有效访问令牌；
- 对源模型、网格、说明和许可证的 ownership rights；
- explicit OneRobotics organizational authorization，可证明有权代表该组织；
- authorized reviewer 对确切内容证据、缩略图、owner、资源名和可见范围的批准。

缺少任意一项就停止。不要用个人判断代替组织授权，不要把真实令牌写入脚本、
配置、测试、命令参数、终端记录或 Git 提交。先生成下面的证据，然后才请审批；
不得在证据尚未生成时先说“已批准”。

先固定一个模型；三个模型必须 one model at a time 分别处理：

~~~bash
set -euo pipefail
source gazebo/scripts/harmonic_env.sh
model_slug='onerobotics_a1_right_arm'
case "$model_slug" in
  onerobotics_a1_right_arm)
    fuel_resource_name='OneRobotics A1 Right Arm'
    fuel_resource_path='OneRobotics%20A1%20Right%20Arm'
    ;;
  onerobotics_a1_left_arm)
    fuel_resource_name='OneRobotics A1 Left Arm'
    fuel_resource_path='OneRobotics%20A1%20Left%20Arm'
    ;;
  onerobotics_a1_bimanual_stand)
    fuel_resource_name='OneRobotics A1 Bimanual Stand'
    fuel_resource_path='OneRobotics%20A1%20Bimanual%20Stand'
    ;;
esac
model_dir="$PWD/dist/gazebo-fuel/$model_slug"
test -d "$model_dir"
release_evidence="$(mktemp -d)"
archive_path="dist/gazebo-fuel/archives/$model_slug-1.0.0.tar.gz"
manifest_path="$release_evidence/$model_slug-1.0.0.manifest.json"
sha256sum "dist/gazebo-fuel/archives/$model_slug-1.0.0.tar.gz" | tee "$release_evidence/$model_slug-1.0.0.archive.sha256"
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.publication manifest \
  --model "$model_dir" \
  --output "$manifest_path"
sha256sum "$manifest_path" | tee "$release_evidence/$model_slug-1.0.0.manifest.sha256"
(cd "$model_dir" && find . -type f -print0 | sort -z | xargs -0 sha256sum) > "$release_evidence/$model_slug-1.0.0.files.sha256"
a1_harmonic_run gz fuel --force-version 9 --versions | tee "$release_evidence/fuel-tools.version.txt"
~~~

现在不要先改 model_slug。每个模型的 release_evidence 都是独立的
`mktemp` 私有目录，不要用新变量覆盖还未验证完的上一个模型。外部的
pre-upload external manifest 必须保存在仓库和待上传目录之外。当前模型的
变量和证据必须一直保留，直到执行完 cp -a。

在请求审批前，在线核对 Fuel 当前仍提供本项目声明的许可证：

~~~bash
set -euo pipefail
/usr/bin/curl --disable --fail --silent --show-error --max-time 30 \
  --proto '=https' --tlsv1.2 --max-filesize 131072 \
  https://fuel.gazebosim.org/1.0/licenses \
  --output "$release_evidence/fuel-licenses.json"
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.publication verify-licenses \
  --response "$release_evidence/fuel-licenses.json" \
  --license "Creative Commons Attribution 4.0 International"
~~~

verify-licenses 会验证 JSON 类型并要求 name 字段精确出现一次；失败就停止，不要
换一个“差不多”的许可证。现在把归档、三张已检查缩略图、manifest 和两个
SHA-256 记录交给审批者。authorized reviewer must approve 完整内容后，应返回：

- 精确的 manifest SHA-256 和归档 SHA-256；
- approved owner, resource name and visibility（private 或 public）；
- 对权利、CC BY 4.0 许可证和已检查缩略图的明确批准。

canonical manifest SHA-256 is the authoritative upload-content identity；归档哈希是另一份
可复现交付证据，不能代替 manifest 哈希。只能把审批者真正返回的值填入
下面五项，不能从当前文件自己重新抄一个值冒充审批：

~~~bash
set -euo pipefail
approved_manifest_sha256='replace-with-authorized-reviewer-approved-manifest-sha256'
approved_archive_sha256='replace-with-authorized-reviewer-approved-archive-sha256'
fuel_owner='replace-with-authorized-reviewer-approved-owner'
approved_resource_name='replace-with-authorized-reviewer-approved-resource-name'
expected_visibility='replace-with-approved-private-or-public'
[[ "$approved_manifest_sha256" =~ ^[0-9a-f]{64}$ ]]
[[ "$approved_archive_sha256" =~ ^[0-9a-f]{64}$ ]]
case "$expected_visibility" in private|public) ;; *) exit 1 ;; esac
test "$approved_resource_name" = "$fuel_resource_name"
printf '%s  %s\n' "$approved_manifest_sha256" "$manifest_path" | sha256sum --check -
printf '%s  %s\n' "$approved_archive_sha256" "$archive_path" | sha256sum --check -
upload_stage_root="$(mktemp -d)"
upload_model_dir="$upload_stage_root/$model_slug"
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.publication prepare-upload \
  --model "$model_dir" \
  --manifest "$manifest_path" \
  --manifest-sha256 "$approved_manifest_sha256" \
  --output "$upload_model_dir"
~~~

这个命令先在同一次稳定读取中强制验证审批的 manifest SHA-256，再把当前模型
与该清单逐项比较，最后只生成一份独立的 read-only 快照。它绑定了真正可交付的
upload_model_dir，不再信任可能继续变化的 model_dir。

### UPLOAD HANDOFF hard stop

环境并没有锁到某个精确小版本：environment-harmonic.yml only constrains gz-fuel-tools9=9.*。
上面保存的版本证据必须随交付物一起审批。the locally audited Fuel Tools 9.1.1 的
[RestClient.cc](https://github.com/gazebosim/gz-fuel-tools/blob/gz-fuel-tools9_9.1.1/src/RestClient.cc)
设置了 `CURLOPT_SSL_VERIFYPEER, 0L`（即 `CURLOPT_SSL_VERIFYPEER=0`）和
`CURLOPT_FOLLOWLOCATION, 1L`（即 `CURLOPT_FOLLOWLOCATION=1`）：它不验证 TLS 对端，并会跟随重定向。
而 all other accepted 9.x clients remain unaudited。因此所有 9.x 版本都不得执行带令牌的网络上传
或下载；本教程故意不提供这个客户端的认证发布命令。

到此必须停止。把 upload_model_dir、外部 manifest 及其已批准 SHA-256、归档 SHA-256、
owner、资源名和可见性交给组织授权维护者。对方只能使用公司批准的安全客户端，
必须验证 TLS 证书和主机名，不得把 `Private-Token` 跟随重定向发给其他 origin，
且只能发布 upload_model_dir。未收到维护者返回的精确 owner、资源名、可见性和“已安全
发布”确认时，不能进入下一节。

## 发布后：真实发布后的下载复验

只有组织授权维护者安全发布并返回与审批值一致的 owner、资源名和可见性后，
才查询 API；不能手填或猜版本。下面仅使用系统 curl：禁用用户配置、不跟随
重定向、验证 TLS，并从标准输入读取 header，令牌不会出现在 curl 命令参数或环境中：

~~~bash
set -euo pipefail
resource_url="https://fuel.gazebosim.org/1.0/$fuel_owner/models/$fuel_resource_path"
fuel_api_json="$release_evidence/$model_slug.resource.json"
fuel_version_file="$release_evidence/$model_slug.version"
test ! -e "$fuel_version_file"
(
  set +x
  trap 'unset GZ_FUEL_TOKEN' EXIT HUP INT TERM
  read -rsp 'Fuel token: ' GZ_FUEL_TOKEN; echo
  printf 'Private-Token: %s\n' "$GZ_FUEL_TOKEN" |
    /usr/bin/curl --disable --fail --silent --show-error --max-time 30 \
    --proto '=https' --tlsv1.2 --max-filesize 131072 --header @- \
    "$resource_url" --output "$fuel_api_json"
)
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.publication verify-api \
  --response "$fuel_api_json" \
  --owner "$fuel_owner" \
  --name "$fuel_resource_name" \
  --license "Creative Commons Attribution 4.0 International" \
  --visibility "$expected_visibility" \
  --version-output "$fuel_version_file"
model_version="$(<"$fuel_version_file")"
raw_zip_url="$resource_url/$model_version/$fuel_resource_path.zip"
raw_check_root="$(mktemp -d)"
(
  set +x
  trap 'unset GZ_FUEL_TOKEN' EXIT HUP INT TERM
  read -rsp 'Fuel token: ' GZ_FUEL_TOKEN; echo
  printf 'Private-Token: %s\n' "$GZ_FUEL_TOKEN" |
    /usr/bin/curl --disable --fail --silent --show-error --max-time 30 \
    --proto '=https' --tlsv1.2 --max-filesize 268435456 --header @- \
    "$raw_zip_url" --output "$raw_check_root/$model_slug-$model_version.zip"
)
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.publication verify-zip \
  --zip "$raw_check_root/$model_slug-$model_version.zip" \
  --manifest "$manifest_path" \
  --manifest-sha256 "$approved_manifest_sha256" \
  --output "$raw_check_root/files"
(cd "$raw_check_root/files" && sha256sum --check "$release_evidence/$model_slug-1.0.0.files.sha256")
verified_models_root="${verified_models_root:-$(mktemp -d)}"
test ! -e "$verified_models_root/$model_slug"
cp -a "$raw_check_root/files" "$verified_models_root/$model_slug"
~~~

上面先用类型化清单安全解析 versioned raw Fuel ZIP，再逐文件检查哈希；只有这份
verified raw ZIP 才会复制到新的本地 staging tree。过程不使用不安全的 Fuel Tools 认证网络
客户端。任何缺失、多余、metadata.pbtxt / model.sdf 不匹配或哈希失败都必须停止。

执行 cp -a 后，若当前还不是第三个模型，回到“发布前：硬性授权门”的 model_slug 设置处，
再回到这里换成下一个 slug，对它独立生成证据、审批、安全交付和复验；始终
保留同一个 verified_models_root。
第三个模型复制完成后才继续。后面的 smoke-cache-all 安全门会比较链接、
关节拓扑和全部来源限制；三个目录齐全后，rerun structural and runtime checks：

~~~bash
set -euo pipefail
test -f "$verified_models_root/onerobotics_a1_right_arm/model.sdf"
test -f "$verified_models_root/onerobotics_a1_left_arm/model.sdf"
test -f "$verified_models_root/onerobotics_a1_bimanual_stand/model.sdf"
env -u PYTHONPATH uv run --project gazebo python -m onerobotics_a1_gazebo.demo smoke-cache-all \
  --trusted-models dist/gazebo-fuel \
  --cache-models "$verified_models_root" \
  --worlds gazebo/worlds
~~~

这次传入 helper 的是 verified raw-ZIP staging tree, not a real Fuel client cache。helper 仍会
先做不可变快照，比较完整文件和 SDF 语义，再复用本地 smoke 的超时、topic 类型、
位置解析和进程清理，对四个代表关节发布小目标：右臂、左臂，以及双臂站架的
`joint_r1` 和 `joint_l1`；每个目标都要求有限位置向目标移动至少 0.02 rad 且不越来源限位。
全部通过时会输出 4 条 CACHE_SMOKE_TEST_VALID 和
`CACHE_SMOKE_TEST_OK: 4 cases across 3 models`。

这一步 does not prove real Fuel client-cache retrieval or URI/XML rewriting。真实客户端缓存复验
remains blocked until an organizationally approved secure client 可用。将来维护者用该客户端
下载后，native Fuel cache root cannot be passed directly to --cache-models：原生缓存有
server / owner / models / resource / version 嵌套，而安全门要求根目录只有三个本地 slug。维护者
必须根据已验证的 owner、资源名和精确版本分别定位三个模型目录，不能用 `find`
猜测；再把它们分别复制为 fresh slug-normalized staging root 下的
`onerobotics_a1_right_arm`、`onerobotics_a1_left_arm` 和 `onerobotics_a1_bimanual_stand`，
最后才把该 staging root 传给 `--cache-models` 重新运行。那时安全门只允许安全的
SDF mesh URI / XML 序列化变化；其余文件和完整 SDF 语义必须与发布前可信包相同。
逐字节发布证据应以前面的 raw ZIP 和 pre-upload external manifest 为准。

## 交付给 mentor 的一句话

可以这样汇报：三个 A1 CAD 修订已经转换成独立的 Gazebo Harmonic / Fuel
本地发布候选，来源锁、结构、SDF、Fuel 元数据、真实运动和缩略图均已验证；
尚未代表 OneRobotics 上传，下一步需要组织授权、Fuel owner、批准的三个
canonical manifest SHA-256 和归档 SHA-256，再由授权维护者使用公司批准的安全客户端发布。
