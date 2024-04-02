#!/bin/bash

package_name=$1
env_name=$2

# 检查是否需要激活虚拟环境
if [ "$2" != "n" ]; then
    conda activate $env_name|| {
        echo "Error: failed to activate $env_name environment"
        exit 1
    }
fi

# 检查包是否已经安装
if python3 -c "import ${package_name}" &> /dev/null; then
    echo "${package_name} has already been installed."
    exit 0
fi

echo "Installing ${package_name}..."

# 安装包
if ! python3 -m pip install -U "${package_name}"; then
    echo "Failed to install ${package_name}."
    exit 2
fi

if [ "$2" != "n" ]; then
    conda deactivate
fi

echo "Done."
exit 0