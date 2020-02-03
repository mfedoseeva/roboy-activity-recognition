# Build

```shell
cd build
cmake ..
# copies opticalflow.so to ../opticalflow
make install
```

# Use

See [test-opticalflow.py](test-opticalflow.py)

```python
import opticalflow.opticalflow as opticalflow

flow = opticalflow.optical_flow(prev_img, curr_img)
```

Or from the directory where opticalflow.so exists:

```python
import opticalflow

flow = opticalflow.optical_flow(prev_img, curr_img)
```

# Credits

[pybind11_opencv_numpy](https://github.com/edmBernard/pybind11_opencv_numpy)


# Dependencies
opencv-cuda  
cuda  
vtk  
cmake (minimum version 3.5)
glew
