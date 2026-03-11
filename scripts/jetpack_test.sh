#!/bin/bash
#
# Jetson Nano CUDA 环境检测脚本
#

echo "========================================="
echo "Jetson Nano CUDA 环境检测"
echo "========================================="
echo ""

# 系统信息
echo "📊 系统信息"
echo "------------------------"
cat /etc/nv_tegra_release
echo ""

# JetPack 版本
echo "🎯 JetPack SDK 版本"
echo "------------------------"
nvpkg --query | grep -i jetpack || echo "未找到 JetPack 信息"
echo ""

# CUDA 版本
echo "⚡ CUDA 版本"
echo "------------------------"
nvcc --version
if [ $? -eq 0 ]; then
    CUDA_VER=$(nvcc --version | grep release | awk '{print $3}' | cut -d',' -f1)
    echo "✅ CUDA 版本: ${CUDA_VER}"
else
    echo "⚠️ nvcc 未安装或不可用"
fi
echo ""

# cuDNN 版本
echo "🧩 cuDNN 版本"
echo "------------------------"
ldconfig -p | grep libcudnn || echo "❌ 未找到 cuDNN"
echo ""

# TensorRT 版本
echo "🤖 TensorRT 版本"
echo "------------------------"
python3 -c "import tensorrt; print(tensorrt.__version__)" 2>/dev/null || echo "⚠️ TensorRT Python 未安装"
echo ""

# PyTorch 支持
echo "🔥 PyTorch CUDA 支持"
echo "------------------------"
python3 -c "import torch; print('CUDA可用:', torch.cuda.is_available())" 2>/dev/null || echo "⚠️ PyTorch 未安装"
if command -v python3 >/dev/null; then
    python3 << EOF
try:
    import torch
    if torch.cuda.is_available():
        print("CUDA版本:", torch.version.cuda)
        print("GPU设备:", torch.cuda.get_device_name(0))
except Exception as e:
    print(f"错误: {e}")
EOF
fi
echo ""

# 路径检查
echo "📁 CUDA 相关路径"
echo "------------------------"
which nvcc || echo "nvcc 未添加到 PATH"
ls /usr/local/cuda* >/dev/null && echo "CUDA路径存在: /usr/local/cuda*" || echo "⚠️ CUDA 安装路径不存在"
echo ""

# 内存使用情况
echo "💾 内存状态"
echo "------------------------"
free -h
echo ""

echo "========================================="
echo "检测完成!"
echo "========================================="