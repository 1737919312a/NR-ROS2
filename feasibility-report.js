const { Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell, 
        Header, Footer, AlignmentType, HeadingLevel, BorderStyle, WidthType, 
        PageNumber, ShadingType, VerticalAlign, LevelFormat, PageBreak } = require('docx');
const fs = require('fs');

// Color Scheme - Midnight Code (Tech style)
const colors = {
  primary: "020617",
  body: "1E293B",
  secondary: "64748B",
  accent: "94A3B8",
  tableBg: "F8FAFC",
  tableHeaderBg: "E2E8F0"
};

const tableBorder = { style: BorderStyle.SINGLE, size: 8, color: colors.secondary };
const cellBorders = { top: tableBorder, bottom: tableBorder, left: { style: BorderStyle.NIL }, right: { style: BorderStyle.NIL } };

const doc = new Document({
  styles: {
    default: { document: { run: { font: "SimSun", size: 21 } } },
    paragraphStyles: [
      { id: "Title", name: "Title", basedOn: "Normal",
        run: { size: 44, bold: true, color: colors.primary, font: "SimHei" },
        paragraph: { spacing: { before: 240, after: 120 }, alignment: AlignmentType.CENTER } },
      { id: "Heading1", name: "Heading 1", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { size: 32, bold: true, color: colors.primary, font: "SimHei" },
        paragraph: { spacing: { before: 360, after: 200 }, outlineLevel: 0 } },
      { id: "Heading2", name: "Heading 2", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { size: 28, bold: true, color: colors.body, font: "SimHei" },
        paragraph: { spacing: { before: 280, after: 160 }, outlineLevel: 1 } },
      { id: "Heading3", name: "Heading 3", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { size: 24, bold: true, color: colors.secondary, font: "SimHei" },
        paragraph: { spacing: { before: 200, after: 120 }, outlineLevel: 2 } }
    ]
  },
  numbering: {
    config: [
      { reference: "bullet-list",
        levels: [{ level: 0, format: LevelFormat.BULLET, text: "\u2022", alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 720, hanging: 360 } } } }] },
      { reference: "num-list-1",
        levels: [{ level: 0, format: LevelFormat.DECIMAL, text: "%1.", alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 720, hanging: 360 } } } }] },
      { reference: "num-list-2",
        levels: [{ level: 0, format: LevelFormat.DECIMAL, text: "%1.", alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 720, hanging: 360 } } } }] },
      { reference: "num-list-3",
        levels: [{ level: 0, format: LevelFormat.DECIMAL, text: "%1.", alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 720, hanging: 360 } } } }] }
    ]
  },
  sections: [{
    properties: {
      page: { margin: { top: 1440, right: 1440, bottom: 1440, left: 1440 } }
    },
    headers: {
      default: new Header({ children: [new Paragraph({ 
        alignment: AlignmentType.RIGHT,
        children: [new TextRun({ text: "智能网联车创新设计比赛 - 可行性分析报告", font: "SimHei", size: 18, color: colors.secondary })]
      })] })
    },
    footers: {
      default: new Footer({ children: [new Paragraph({ 
        alignment: AlignmentType.CENTER,
        children: [
          new TextRun({ text: "第 ", font: "SimSun", size: 18 }), 
          new TextRun({ children: [PageNumber.CURRENT], font: "SimSun", size: 18 }), 
          new TextRun({ text: " 页 / 共 ", font: "SimSun", size: 18 }), 
          new TextRun({ children: [PageNumber.TOTAL_PAGES], font: "SimSun", size: 18 }), 
          new TextRun({ text: " 页", font: "SimSun", size: 18 })
        ]
      })] })
    },
    children: [
      // Title
      new Paragraph({ heading: HeadingLevel.TITLE, children: [new TextRun("智能网联车创新设计比赛")] }),
      new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 400 },
        children: [new TextRun({ text: "ROS 2 云边端协同架构可行性分析报告", font: "SimHei", size: 32, color: colors.secondary })] }),
      
      // Section 1
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun("一、项目概述")] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("1.1 比赛要求总结")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "本次比赛围绕\"车-路-云\"一体化协同系统展开，要求构建\"仿真平台 + 智能小车 + 实景场地\"的虚实融合验证环境。比赛提供两个参赛方向：\"编队行驶与车车协同\"以及\"机器视觉与感知融合\"，共同探索5G网络环境下智能网联汽车的协同与控制能力。实景场地尺寸为3米×4.5米，这一有限空间对系统的精度控制提出了较高要求。", font: "SimSun", size: 21 })] }),
      
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "基本达标要求包含四个核心指标：首先，车辆需至少完成1次十字路口通行；其次，车路协同至少实现交通灯识别或基于V2X通信中的一种方式；第三，车辆行驶过程中中间发生碰撞不超过2次；最后，仿真平台至少呈现1个网络性能指标项。这些要求既考验了系统的感知能力，也验证了通信与控制的可靠性。", font: "SimSun", size: 21 })] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("1.2 技术方案概览")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "提交的ROS 2云边端协同架构采用分层设计，包含边缘端、感知执行层和云端三个核心层级。边缘端以Raspberry Pi 4B为核心，运行Micro-ROS Agent汇聚数据，同时承担OpenCV车道保持、AprilTag从机定位和5G网关视频推流等关键任务。感知执行层由两个ESP32-S3 MCU组成，主机MCU负责主机的传感器融合与电机PID控制，从机MCU负责本地激光避障与跟随执行。云端部分则运行YOLO进行语义认知，识别红绿灯、行人等目标，并下发高层决策指令。", font: "SimSun", size: 21 })] }),
      
      // Section 2
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun("二、需求匹配度分析")] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("2.1 与\"编队行驶与车车协同\"方向的匹配度")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "该方向要求实现多车编队协同控制，重点研究基于NR-V2X通信的车车协同决策与智能驾驶。技术方案中的主从机系统设计与此方向高度契合：主机通过AprilTag视觉定位从机位置，从机根据接收的跟随指令执行运动，这正是编队控制的核心实现。系统支持虚实融合验证，可以在仿真平台实时同步编队运行状态。", font: "SimSun", size: 21 })] }),
      
      // Matching table
      new Paragraph({ spacing: { before: 200, after: 100 }, alignment: AlignmentType.CENTER,
        children: [new TextRun({ text: "表 1：编队行驶方向需求匹配表", font: "SimHei", size: 20, bold: true })] }),
      
      new Table({
        columnWidths: [2400, 3200, 3200],
        alignment: AlignmentType.CENTER,
        margins: { top: 80, bottom: 80, left: 120, right: 120 },
        rows: [
          new TableRow({
            tableHeader: true,
            children: [
              new TableCell({ borders: cellBorders, shading: { fill: colors.tableHeaderBg, type: ShadingType.CLEAR }, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "需求项", font: "SimHei", size: 20, bold: true })] })] }),
              new TableCell({ borders: cellBorders, shading: { fill: colors.tableHeaderBg, type: ShadingType.CLEAR }, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "技术方案支持", font: "SimHei", size: 20, bold: true })] })] }),
              new TableCell({ borders: cellBorders, shading: { fill: colors.tableHeaderBg, type: ShadingType.CLEAR }, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "匹配度评价", font: "SimHei", size: 20, bold: true })] })] })
            ]
          }),
          new TableRow({
            children: [
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "多车编队控制", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "主从机系统 + AprilTag定位", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "★★★★ 良好", font: "SimSun", size: 20, color: "059669" })] })] })
            ]
          }),
          new TableRow({
            children: [
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "动态避障", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "从机LD06激光避障", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "★★★★ 良好", font: "SimSun", size: 20, color: "059669" })] })] })
            ]
          }),
          new TableRow({
            children: [
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "NR-V2X通信", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "5G + WiFi UDP替代方案", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "★★★ 一般", font: "SimSun", size: 20, color: "D97706" })] })] })
            ]
          }),
          new TableRow({
            children: [
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "仿真同步", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "云边端数据同步", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "★★★★★ 优秀", font: "SimSun", size: 20, color: "059669" })] })] })
            ]
          })
        ]
      }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("2.2 与\"机器视觉与感知融合\"方向的匹配度")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "该方向侧重机器视觉在\"车-路-云\"协同系统中的应用，要求实现车道线识别与跟踪、交通灯识别与响应、行人检测与避障等视觉环境感知功能。技术方案中的OpenCV车道保持、云端YOLO目标检测等模块与此方向完全契合。系统能够将感知结果与车辆位姿在仿真平台中实时同步展示，符合比赛要求。", font: "SimSun", size: 21 })] }),
      
      // Section 3
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun("三、技术可行性评估")] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("3.1 硬件平台可行性")] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun("3.1.1 边缘端 (Raspberry Pi 4B)")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "Raspberry Pi 4B (4GB RAM) 作为边缘计算平台具有较高的可行性。经过超频至2.2GHz并配备主动散热后，计算性能可满足OpenCV车道检测和AprilTag定位的实时性要求。但需要注意的是，同时运行Micro-ROS Agent、视觉处理、视频编码推流等多个任务时，内存和CPU资源可能成为瓶颈。建议采用多线程执行器和ROS 2组件组合技术进行优化，以提高并发性能和降低延迟。", font: "SimSun", size: 21 })] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_3, children: [new TextRun("3.1.2 MCU平台 (ESP32-S3)")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "ESP32-S3 搭载Xtensa LX7双核处理器，主频2.4GHz，具备丰富的外设接口和低功耗特性，非常适合作为机器人的底层控制器。其双核架构可以分别处理传感器数据采集和运动控制，满足实时性要求。该平台对Micro-ROS的支持较好，可以方便地与ROS 2系统集成。然而，ESP32-S3的内存资源相对有限，在处理激光雷达数据时需要精心设计缓冲区管理策略，以避免内存溢出问题。", font: "SimSun", size: 21 })] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("3.2 软件架构可行性")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "ROS 2 Humble版本提供了稳定的DDS通信基础设施，支持实时性通信和分布式部署。Micro-ROS框架允许ESP32-S3以节点方式接入ROS 2网络，实现透明的数据交换。OpenCV和AprilTag库在嵌入式平台上有成熟的应用案例，开发难度相对较低。云端YOLOv8模型在RTX 3060以上GPU可达100+ FPS，能够满足实时检测要求。整体软件栈技术成熟度较高，开发风险可控。", font: "SimSun", size: 21 })] }),
      
      // Section 4
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun("四、面临的主要问题与挑战")] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("4.1 通信体系差异问题")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "比赛要求基于NR-V2X通信实现车车协同，但技术方案采用的是5G + WiFi UDP的替代方案。这存在以下差异：首先，NR-V2X支持车车直连通信，延迟可达毫秒级，而WiFi UDP在经过5G网络转发后延迟明显增加；其次，NR-V2X具有专用的QoS保障机制，而通用IP网络需要自行实现可靠性保障；第三，比赛评分标准可能对NR-V2X有特殊要求，需要确认其是否为强制要求。", font: "SimSun", size: 21 })] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("4.2 实景场地尺寸限制")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "3米×4.5米的实景场地对系统提出了严峻挑战。这一尺寸对于多车编队运行来说相对紧凑，要求系统具备极高的定位精度和控制精度。LD06激光雷达的有效检测距离为12米，在小尺寸场地中可能出现近距离盲区问题。主从机之间的安全距离控制也需要精确调优，以避免碰撞。建议进行多次实地测试，确定最优的跟随距离和安全阈值。", font: "SimSun", size: 21 })] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("4.3 十字路口通行挑战")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "基本达标要求车辆至少完成1次十字路口通行，这涉及多个关键技术环节。系统需要实现精准的车道线检测和跟踪，能够在路口正确转弯或直行。交通灯识别和响应机制需要与车辆控制系统紧密协调，实现红灯停车、绿灯通行的自动化控制。同时，云端决策与边缘端执行之间的延迟也可能影响路口通行的实时性，需要设计合理的延迟补偿机制。", font: "SimSun", size: 21 })] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("4.4 网络延迟与可靠性")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "5G网络在室内环境中的覆盖和信号强度可能不稳定，导致视频推流和指令下发出现抖动或中断。视频流的编解码延迟加上网络传输延迟，可能导致云端收到的画面与实际情况存在时间差，影响决策的及时性。从机通过WiFi连接5G CPE再连接主机的通信链路较长，任何一个环节的故障都可能导致从机失控。必须设计完善的故障容错和安全降级机制。", font: "SimSun", size: 21 })] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("4.5 资源竞争与性能瓶颈")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "Raspberry Pi 4B需要同时运行多个计算密集型任务，包括视觉处理、视频编码、通信代理等。这些任务之间存在CPU和内存资源竞争，可能导致处理延迟增加或丢帧。USB 3.0和CSI摄像头同时工作时，可能产生带宽竞争，影响图像采集的稳定性。建议进行详细的性能分析和任务调度优化，确保各任务获得足够的计算资源。", font: "SimSun", size: 21 })] }),
      
      // Section 5
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun("五、解决方案与建议")] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("5.1 通信方案调整建议")] }),
      new Paragraph({ numbering: { reference: "num-list-1", level: 0 }, spacing: { line: 312 },
        children: [new TextRun({ text: "与比赛组委会确认NR-V2X是否为强制要求。如果只是参考方向，当前的5G + WiFi方案可以接受；如果是强制要求，需要采购NR-V2X模块或调整技术方案。", font: "SimSun", size: 21 })] }),
      new Paragraph({ numbering: { reference: "num-list-1", level: 0 }, spacing: { line: 312 },
        children: [new TextRun({ text: "优化WiFi UDP通信的可靠性。添加序号、校验和确认机制，确保关键指令的可靠传输。实现自动重传机制，对丢包进行补偿。", font: "SimSun", size: 21 })] }),
      new Paragraph({ numbering: { reference: "num-list-1", level: 0 }, spacing: { line: 312 },
        children: [new TextRun({ text: "实现边缘端本地决策与云端决策的协同。当网络不稳定时，边缘端可以独立运行基于本地传感器的安全控制，不依赖云端指令。", font: "SimSun", size: 21 })] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("5.2 系统性能优化建议")] }),
      new Paragraph({ numbering: { reference: "num-list-2", level: 0 }, spacing: { line: 312 },
        children: [new TextRun({ text: "使用MultiThreadedExecutor并配置合理的回调组策略。传感器回调使用ReentrantCallbackGroup提高并发性，控制回调使用MutuallyExclusiveCallbackGroup保证一致性。", font: "SimSun", size: 21 })] }),
      new Paragraph({ numbering: { reference: "num-list-2", level: 0 }, spacing: { line: 312 },
        children: [new TextRun({ text: "启用ROS 2 Composition和IPC Zero-Copy技术。这需要将关键节点用C++重写，减少进程间通信的数据复制开销。", font: "SimSun", size: 21 })] }),
      new Paragraph({ numbering: { reference: "num-list-2", level: 0 }, spacing: { line: 312 },
        children: [new TextRun({ text: "优化视觉处理流程。使用GPU加速的OpenCV操作，降低图像处理延迟。对于车道检测，可以考虑使用ROI裁剪减少处理区域。", font: "SimSun", size: 21 })] }),
      new Paragraph({ numbering: { reference: "num-list-2", level: 0 }, spacing: { line: 312 },
        children: [new TextRun({ text: "合理分配任务优先级。安全相关任务（如避障）应具有最高优先级，视觉处理和通信任务根据实际负载进行动态调整。", font: "SimSun", size: 21 })] }),
      
      new Paragraph({ heading: HeadingLevel.HEADING_2, children: [new TextRun("5.3 安全机制加强建议")] }),
      new Paragraph({ numbering: { reference: "num-list-3", level: 0 }, spacing: { line: 312 },
        children: [new TextRun({ text: "实现多级看门狗机制。硬件看门狗作为最后防线，软件看门狗监控所有关键通信链路，通信超时立即触发安全停车。", font: "SimSun", size: 21 })] }),
      new Paragraph({ numbering: { reference: "num-list-3", level: 0 }, spacing: { line: 312 },
        children: [new TextRun({ text: "设计完善的故障降级策略。当云端连接丢失时，系统切换到边缘端本地控制模式；当从机通信中断时，从机执行安全停车。", font: "SimSun", size: 21 })] }),
      new Paragraph({ numbering: { reference: "num-list-3", level: 0 }, spacing: { line: 312 },
        children: [new TextRun({ text: "强化从机本地避障逻辑。从机的本地避障应具有最高优先级，任何时候检测到前方障碍物都应立即停车，不受远程指令影响。", font: "SimSun", size: 21 })] }),
      
      // Section 6
      new Paragraph({ heading: HeadingLevel.HEADING_1, children: [new TextRun("六、总体可行性评价")] }),
      new Paragraph({ indent: { firstLine: 420 }, spacing: { line: 312 },
        children: [new TextRun({ text: "综上所述，提交的ROS 2云边端协同架构整体上具有较高的可行性，能够满足比赛的大部分要求。系统采用的分层架构设计合理，边缘端、MCU层和云端的职责划分清晰，技术选型成熟可靠。然而，在NR-V2X通信协议、小尺寸场地适配、网络延迟控制等方面仍面临一定挑战，需要在开发过程中重点关注和解决。", font: "SimSun", size: 21 })] }),
      
      // Feasibility Summary Table
      new Paragraph({ spacing: { before: 200, after: 100 }, alignment: AlignmentType.CENTER,
        children: [new TextRun({ text: "表 2：整体可行性评价汇总", font: "SimHei", size: 20, bold: true })] }),
      
      new Table({
        columnWidths: [2800, 2800, 3200],
        alignment: AlignmentType.CENTER,
        margins: { top: 80, bottom: 80, left: 120, right: 120 },
        rows: [
          new TableRow({
            tableHeader: true,
            children: [
              new TableCell({ borders: cellBorders, shading: { fill: colors.tableHeaderBg, type: ShadingType.CLEAR }, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "评估维度", font: "SimHei", size: 20, bold: true })] })] }),
              new TableCell({ borders: cellBorders, shading: { fill: colors.tableHeaderBg, type: ShadingType.CLEAR }, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "可行性评级", font: "SimHei", size: 20, bold: true })] })] }),
              new TableCell({ borders: cellBorders, shading: { fill: colors.tableHeaderBg, type: ShadingType.CLEAR }, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "主要风险/注意事项", font: "SimHei", size: 20, bold: true })] })] })
            ]
          }),
          new TableRow({
            children: [
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "硬件平台", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "★★★★ 良好", font: "SimSun", size: 20, color: "059669" })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "资源竞争、散热问题", font: "SimSun", size: 20 })] })] })
            ]
          }),
          new TableRow({
            children: [
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "软件架构", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "★★★★★ 优秀", font: "SimSun", size: 20, color: "059669" })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "需注意性能优化", font: "SimSun", size: 20 })] })] })
            ]
          }),
          new TableRow({
            children: [
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "通信方案", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "★★★ 一般", font: "SimSun", size: 20, color: "D97706" })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "NR-V2X替代方案差异", font: "SimSun", size: 20 })] })] })
            ]
          }),
          new TableRow({
            children: [
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "现场适配", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "★★★ 一般", font: "SimSun", size: 20, color: "D97706" })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "小尺寸场地精度控制", font: "SimSun", size: 20 })] })] })
            ]
          }),
          new TableRow({
            children: [
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "基本达标能力", font: "SimSun", size: 20 })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "★★★★ 良好", font: "SimSun", size: 20, color: "059669" })] })] }),
              new TableCell({ borders: cellBorders, verticalAlign: VerticalAlign.CENTER,
                children: [new Paragraph({ alignment: AlignmentType.CENTER, children: [new TextRun({ text: "十字路口、避障测试", font: "SimSun", size: 20 })] })] })
            ]
          })
        ]
      }),
      
      new Paragraph({ indent: { firstLine: 420 }, spacing: { before: 300, line: 312 },
        children: [new TextRun({ text: "建议在正式比赛前进行充分的系统测试和优化，特别关注网络延迟、定位精度和安全机制等关键指标。同时，建议准备多套应急预案，以应对比赛现场可能出现的各种突发情况。通过合理的技术选择和细致的实施规划，该项目有望在比赛中取得优异成绩。", font: "SimSun", size: 21 })] })
    ]
  }]
});

Packer.toBuffer(doc).then(buffer => {
  fs.writeFileSync("/home/z/my-project/download/ROS2-Feasibility-Analysis-Report.docx", buffer);
  console.log("Report generated: /home/z/my-project/download/ROS2-Feasibility-Analysis-Report.docx");
});
