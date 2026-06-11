The following assumes a JetPack version of 6.2.1 and Python version 3.10.12, CUDA 12.6

(ORBBEC INSTALL)

Second, ultralytics must be installed via:

pip install ultralytics 

This will likely install an incompatible x86 PyTorch version, where then you must run:

pip uninstall torch torchvision 
pip innstall torch==2.8.0 torchvision==0.23.0 --index-url https://pypi.jetson-ai-lab.io/jp6/cu126

In order to install the jetson-compatible wheel for torch 2.80, torchvision 0.23.0 

All other required pip packages may be inferred via imports in the code, and can be installed directly. 
