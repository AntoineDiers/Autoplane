

apt update
apt install -y git libclang-dev python3-pip python3-vcstool python3-venv
cd /
python3 -m venv python_venv
. /python_venv/bin/activate 
pip install git+https://github.com/colcon/colcon-cargo.git
pip install git+https://github.com/colcon/colcon-ros-cargo.git