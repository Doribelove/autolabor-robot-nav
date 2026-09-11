# AI连续任务修复与模拟验证（2026-09-12）

## 结论与范围

分支为`robot_j6m_ws_vision_0911`，仅在本工作区继续修改。已修复上一轮模拟发现的
返回点采集触发受限、完整动作前半句过早提交、视觉退出误报成功三个问题。
新增测试又发现云端可能给未指定区域的请求默认选A、把人的位置当机器人位置，现分别增加
窄范围本地校验和硬性预检提示；不改动态区域通用语义匹配为固定别名。

最终真实云端25/25，隔离执行/异常/停顿47/47；AI206、视觉84、Qt52项单测/契约通过，
真实视觉节点11项合成传感器ROS场景通过，Release/静态/暂存安装检查通过。
这是本地候选验证，不是实车或真实音频验收；尚未部署、重启或提交/推送。

最低指令：**“清扫A区后回到当前点进行视觉清扫”**。

```text
只读记录本轮map位姿与地图摘要
  → 覆盖清扫A并等完整结束/释放所有权
  → 导航回记录点并等同一GoalID成功
  → 原FOD视觉伺服
  → 同一视觉会话的新完成回执 + 停车 + 返回导航模式，才认定完成
```

真实目录的区域名为`A`，不为了测试新增`A区`。云端提交的是后面三个动作，位姿记录是
本地规划前置步骤。模拟原位置map(3.125,-4.25,37.5°)，覆盖后机器人位置变为(40,20,-90°)，
返回目标仍严格使用原冻结点及朝向，不能在清扫后重新取“当前位置”。

## 修复方法

### 1. 先提供可信记录，再做通用语义理解

每个新请求在云端规划前尝试`get_robot_status(capture_map_pose=true)`，不再只在命中
几个返回短语时记录。独立短停止/取消/暂停不增加位姿采集等待，澄清补答保留本轮记录。
不需要返回的普通任务不附加返回点执行门；快照存在不构成模糊区域或任意导航的依据。

保持已定位、新鲜map->base_link、有限值和地图摘要一致性校验。“现在这儿/这个位置/
原地/机器人目前停着的位置/记住现在位置做完回来”等通过通用规则解析。“我现在的位置”
指人，“刚才的位置”指未保存历史点，不能拿当前机器人位姿冒充；缺信息整单空计划询问。
仅说“去区域清扫”且无区域上下文也不允许按目录顺序选A。原点仍默认map(0,0,0°)。

### 2. 有界动作续句窗口

`VoiceTurnBuffer`默认：查询/补答1.5秒、动作句3秒、连续动作或未完句4秒、孤立短片段6秒过期。
新增`asr.turn_command_quiet_s=3.0`，私有配置未改，缺省自动使用新默认。
识别出“先清扫A区”虽是完整动作，仍依据“先”保留4秒续句窗口；后续返回/视觉片段合并
为同轮请求。继续等待后续音频采集/识别，保留3轮60秒上下文、噪声/短片段过滤、取消清空。
“说完了”立即提交，停止/取消立即处理，不受动作等待影响。

代价是独立动作相对旧版多等约1.5秒，连续动作多等约2.5秒；窗口从采集时钟计算，实际
响应还受ASR排队/解码影响。超过3/4秒仍可能新分轮，不能承诺任意长停顿或因此提升ASR准确率。

### 3. 视觉完成绑定会话与停车证据

原问题是AI只看“活动后回导航模式”，而J6M原`last_cleaning_completion`只覆盖选定垃圾
任务，不足以证明普通视觉清扫完成。现在在视觉节点增加纯结果元数据`VisualSessionReceipt`：

- 每次实际启用分配新UUID，result为RUNNING；不是AI根据历史最新状态猜测身份。
- 原传感器复核、最终停车确认后的COMPLETE提交处生成新完成序号和stopped=true。
- COMPLETE回执跨DISABLED保留；ABORT和主动退出分别记录ABORT/CANCELLED，不产生成功回执。
- 新会话不能复用旧ID/旧序号。原选定垃圾完成证据继续保留，视觉动作与5cm停车保护不改。

状态增加`ai_completion_protocol=1`及`visual_session`：

```json
{
  "session_id": "本次32位小写十六进制UUID",
  "result": "RUNNING / COMPLETE / ABORT / CANCELLED / IDLE",
  "sequence": 1,
  "last_completion": {
    "session_id": "完成会话的UUID",
    "sequence": 1,
    "result": "COMPLETE",
    "stopped": true
  }
}
```

初始ID为空、sequence为0、last_completion为空；图示不是JSON schema。
AI启用前检查协议、视觉停用和模式就绪，记录基线；启用后确认新ID。
只有同ID、新序号、result=COMPLETE、stopped=true、视觉COMPLETE/DISABLED且active=false、
仲裁恢复导航模式才成功。查询报错、模式/状态丢失、ABORT、手动退出、旧回执和会话替换
不能放行下一步。

失败只尝试退出仍匹配的本AI会话。服务返回丢失时继续尝试确认新ID；能确认则按ID清理，
不能确认则保留占用并阻止AI继续导航/覆盖，不擅停外部任务。服务发现阶段尚未发请求的
失败不留下占用，明确无目标空操作不算成功。取消发生在启用回执到达前时，回执到达后
补做本AI会话退出；UI总结不再无证据声称“已经停车”。

现有SetBool服务本身不携带事务ID；状态完全丢失或外部并发接管等无法证明归属的情况
仍保守阻断，需要操作员按原统一流程排查。没有新增绕过安全门或全局停止其他任务的逻辑。

## 验证结果与证据

证据根目录：`validation/ai_control_repair_20260912/`。

| 层次 | 最终结果 | 证据/覆盖范围 |
|---|---|---|
| 真实云端基础 | 12/12 | `cloud_base_verified.jsonl`，链任务、仅覆盖、原点、缺区域、人的位置、历史点、未定位 |
| 真实云端扩展 | 8/8 | `cloud_extended_verified.jsonl`，7种返回表达及1个多区域链 |
| 原话重复 | 5/5 | `cloud_minimum_verified.jsonl`，每次只读采集1次、匹配真实A、3步顺序不变 |
| 隔离执行 | 47/47 | `scenarios_final.jsonl`：17个控制场景、10组停顿、2个四步对照、18份最终云端三段计划回放 |
| AI/MCP/ASR单测 | 206/206 | `regression_verified.log`的11个独立进程，最后变更再跑`final_delta_tests.log`，不重复累加 |
| 视觉控制单测 | 84/84 | `regression_verified.log`，原控制/目标/传感器/停车判据 |
| Qt契约 | 52/52 | 同上，非实际窗口/点击验收 |
| 真实视觉节点ROS | 11/11 | `visual_ros_final.log`及`ros_results_final/`，完整普通/选点流程、COMPLETE回执跨停用保存、ABORT无本次回执 |
| 构建/检查 | 通过 | `build_final.log`、`static.log`、`staged_install_verified.log` |

25次云端均一次响应，平均2.00秒、最长2.70秒；真实区域目录和云端，位姿为内存合成，
MCP只提供两个只读方法，变更调用会抛异常。耗时不是实车TF采集/ASR总延迟。
18份链计划在真实AgentRunner/工具注册表/授权回调和真实ROS视觉状态适配方法上回放。
多区域例只核对云端计划，不算这18份执行回放。四步对照明确验证“视觉未完成不得继续导航”，
也验证正常完成后可以继续第四步，并非通过一律拒绝获得安全测试通过。

执行模拟和真实视觉节点测试均通过`optimized.sh`再进入只有lo的网络/PID命名空间，
最小/dev，无实车ROS网络、麦克风、底盘设备或真实速度输出。真实节点仅消费合成检测、
里程计、轮速和CAN状态。原故障注入触发ABORT属于预期通过，不等于实车故障已经修复。
静态检查的639是已有catkin XML汇总，不是本次独立新测639项。

中间失败保留：动作等待改变后旧5项时序断言、mock缺visual_state、错误Qt测试路径、
新增断言预期异常类型错误；已修正并重新回归。第一轮基础云端11/12擅选未指定区域，
第二轮11/12仍把人的位置当机器人位置（被本地拒绝），最终硬性提示后12/12。
首次ROS运行因结果目录传相对路径导致汇总找不到XML，不计通过；改绝对路径重跑11/11。
首次暂存安装使用错误cmake路径失败，改为构建实际目录后安装/独立导入通过。
历史上一轮49场景42通过/7失败仍保留于[原报告](AI_CONTROL_SIMULATION_20260912.md)。

## 复现

云端测试私下读取已有密钥，日志不输出密钥，不启动机器人：

```bash
./scripts/optimized.sh run bash -c 'source scripts/setup_env.sh; python3 scripts/check_ai_return_plans.py'
./scripts/optimized.sh run bash -c 'source scripts/setup_env.sh; python3 scripts/check_ai_return_plans.py --minimum'
./scripts/optimized.sh run bash -c 'source scripts/setup_env.sh; python3 scripts/check_ai_return_plans.py --extended'
```

离线控制回放必须保留网络隔离，不能连接真实master：

```bash
./scripts/optimized.sh run bwrap --unshare-net --unshare-pid --die-with-parent \
  --ro-bind / / --bind "$PWD" "$PWD" --tmpfs /tmp --proc /proc --dev /dev -- bash -c '
    source scripts/setup_env.sh
    unset ROS_HOSTNAME
    export ROS_MASTER_URI=http://127.0.0.1:11496 ROS_IP=127.0.0.1
    python3 scripts/simulate_ai_control.py \
      --cloud-log validation/ai_control_repair_20260912/cloud_base_verified.jsonl \
      --cloud-log validation/ai_control_repair_20260912/cloud_minimum_verified.jsonl \
      --cloud-log validation/ai_control_repair_20260912/cloud_extended_verified.jsonl
  '
```

## 部署与未验收边界

本次NVIDIA AI代码及J6M视觉结果元数据均有修改，需要配套部署J6M，再经统一生命周期
完整启动；仅重启Qt不够。部署脚本增加安装模块导入检查，未改共享ROS服务定义/MD5。
旧板端没有`ai_completion_protocol=1`时AI在启用前拒绝，不会以旧终态判据执行视觉。

原综合页导航、所有权桥、覆盖/TEB、底盘和视觉运动策略未因本任务修改；工作树原有
待清扫、Qt和另项覆盖自动重试修改全部保留。本轮未部署、未开启/关闭实车服务或Qt、
未发送真实初值/目标/速度、未调整运动/AI授权、未录音/播放或重新跑Whisper模型。
`protected_before.sha256`对照确认dual_host私有配置、主运动标记和真实区域文件一致，
未新增测试区域。源码仍在工作树，未提交/推送。

上一段记录的是修复完成时的状态。2026-09-12 01:16，用户随后明确要求部署整个工作空间。
统一入口先确认覆盖任务已完成、最终速度为零，再完整停止旧板端会话；J6M原生release
`20260912_011633`部署成功，视觉节点包含`ai_completion_protocol=1`和`VisualSessionReceipt`，
关键文件与本地SHA256一致，共享服务两端MD5均为
`0c6be3379e5f6ea397f1cbc11681d36b`。NVIDIA侧全工作空间Release构建也已通过，Qt、AI/MCP
和视觉产物更新到当前`devel`。部署后两端保持停止，J6M无11311监听；尚未完整启动、设置
新初值或进行实车连续任务验收。证据见
`validation/full_workspace_deploy_20260912/release_20260912_011633/`。
