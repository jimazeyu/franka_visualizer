# franka_visualizer
Visualize franka arm FK and IK.

## installation
```bash
conda create -n franka_visualizer python=3.8
conda activate franka_visualizer
pip install -r requirements.txt
```

## usage 
- Run 'python main.py', and follow the GUI.
- Every time IK is executed 20 times and i use the one which is the closet to the present joint position. 

# example
<p align="center">
	<img src="./assets/example.png">
</p>
