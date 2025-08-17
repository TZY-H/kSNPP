## 📄 中文 README

# CDT-TetheredPathPlanning

这是一个用于2D移动机器人快速规划k条最短非同伦路径的库。

## 🧩 功能模块

- **CDT-kSNP**
  快速规划k条最短非同伦路径
  实现于：`BImap::kSNPPlanner`

- **CDT-kSNP2THCS**
  快速找到k条(或所有)系留机器人的最短非同伦配置
  实现于：`BImap::kSNP2THCSearcher`


## ⚙️ 运行要求

### Python 部分
在运行测试程序前，请使用安装了 **OpenCV 3.4.9** 的 **Python3** 执行以下命令启动多边形拟合服务：
```bash
python3 approx_work.py
```

### C++ 编译依赖
- OpenCV 4.0 或更高版本

## 📁 示例工程说明

`./test/` 目录下包含3个示例程序：

- **CDT-kSNP 无交互界面性能测试程序**：
  - `expKSNPP.cpp`

- **CDT-kSNP 有交互界面测试程序**：
  - `testKSNPP.cpp`

- **CDT-kSNP2THCS 有界面测试程序**：
  - `testTHCS.cpp`

## 🚀 使用方法

1. 启动 Python 多边形拟合服务：
   ```bash
   python3 approx_work.py
   ```

2. 编译 C++ 代码并运行相应的测试程序。

## 💡 注意事项

- 确保所有依赖项正确安装。
- 若需调试或扩展功能，请参考源码中的类与函数定义。

---

