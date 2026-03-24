# echo "🔧 安装nvcc环境..."
sudo apt-get update
sudo apt-get install -y nvidia-jetpack

command -v nvcc || true
ls -l /usr/local/cuda*/bin/nvcc 2>/dev/null || true

source venv/bin/activate

# 清理旧的/CPU版
python -m pip uninstall -y torch torchvision torchaudio


# 安装 JP6 + cu126 的 GPU 版
python -m pip install -U pip
python -m pip install torch torchvision --index-url https://pypi.jetson-ai-lab.io/jp6/cu126

# 1) 安装 CUDA 仓库 keyring
sudo apt-get update
sudo apt-get install -y wget ca-certificates gnupg
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb

# 2) 更新并安装 cuDSS（CUDA 12.6 对应）
sudo apt update
sudo apt install -y libcudss0-cuda-12
sudo ldconfig

echo "/usr/lib/aarch64-linux-gnu/libcudss/12" | sudo tee /etc/ld.so.conf.d/libcudss.conf
sudo ldconfig
ldconfig -p | grep -i cudss

python -c "import torch; print(torch.__version__); print('torch.version.cuda =', torch.version.cuda); print('cuda available =', torch.cuda.is_available())"
