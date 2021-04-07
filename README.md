# Nonlinear MIMO Controller for Generalized Soft Robots
### by Sarah Aguasvivas Manzano

- Step 1: Learn the forward dynamics
- Step 2: Train the feedforward MLP
- Step 3: Configure Controller
- Step 4: Run! 


**To cite this paper:**

We encourage our users to cite the publication that we prepared for this work. Please find it at this direction:
```
@misc{manzano2021highbandwidth,
      title={High-bandwidth nonlinear control for soft actuators with recursive network models}, 
      author={Sarah Aguasvivas Manzano and Patricia Xu and Khoi Ly and Robert Shepherd and Nikolaus Correll},
      year={2021},
      eprint={2101.01139},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

**The model:** 
The model is a hierarchical, recursive, neural network model that is capable of doing forward predictions by feeding back to itself the predictions from previous timesteps. Here ![formula](https://render.githubusercontent.com/render/math?math=\mathbf{y}_{t}) is the output of the system. 

![alt text](https://github.com/sarahaguasvivas/nlsoft/blob/master/docs/neural_network_architecture.gif "Neural Network Architecture")


![alt text](https://github.com/sarahaguasvivas/nlsoft/blob/master/docs/system_description.gif "Description of the System")
