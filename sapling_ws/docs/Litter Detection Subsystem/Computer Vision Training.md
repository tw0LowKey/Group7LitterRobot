## Computer Vision Training

Should a new computer vision model be needed to be trained, the file to download and split TACO into train/test/val is stored at `misc/Litter Detection Subsystem/computer_vision_training.ipynb`. This file should be used to generate an ONNX file, which once installed onto the Jetson Orin Nano may be converted to a Jetson architecture-specific .engine file using TensorRT. 

The file `computer_vision_training.ipynb` also allows ultralytics-specific models to be swapped out as newer models are created, and allows parameters to be tuned. An entirely different dataset could also be created/used to cover gaps in the TACO dataset, but this would require a different dataset import structure. 

Once the engine file is created, it should be add to `src/domains/vision/computer_vision/models`, as well as the `setup.py`, and swapped out in `vision/computer_vision/computer_vision/rgb_depth_node.py` at the line:

```python
model_path = os.path.join(
            get_package_share_directory('computer_vision'),
            'models', 'Model_1_engine_2.engine'
)
```
