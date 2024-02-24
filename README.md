# Car2BALL

## Prerequisites:

1. [Unity](https://unity.com/download) 2022.3 or later
2. Python 3.10.13  
3. Torch 1.13.1 (default in environment.yml is the cpu version but feel free to change it to a gpu version provided you have CUDA support)
3. MLAgents release-20 branch
4. mlagents and mlagents-envs python packages (installed from mlagents cloned reporsitory)

#### For easy installation of python related stuff - use conda and install a new environment with the environment.yml file in this repo. This installs required versions of python, pytorch and other dependencies EXCEPT mlagents. mlagents packages need to be installed directly from the cloned mlagents repo.

### To continue with Unity mlagents package installation: 

You can follow the official ml-agents setup guide from unity [here](https://unity-technologies.github.io/ml-agents/Installation/). Note that the python version we use is 3.10.13 instead of 3.10.12. The ml-agents repository that you clone should be --branch release_20 instead of --branch release_21 (due to some compatibility issues with Numpy.) 

```
git clone --branch release_20 https://github.com/Unity-Technologies/ml-agents.git
```

*IMPORTANT*: Now you have to replace 2 scripts in the cloned repo from our repo. 

*Car2Ball/setup_replacement/ml-agents/setup.py -> ml-agents/ml-agents/setup.py*
*Car2Ball/setup_replacement/ml-agents-envs/setup.py -> ml-agents/ml-agents-envs/setup.py*

Only after you replace the setup scripts, open a conda terminal and run the below commands inside the conda environment (mlagents) you installed from environment.yml.

```
conda activate mlagents
cd /path/to/ml-agents
python -m pip install ./ml-agents-envs
python -m pip install ./ml-agents
```

Then add the path to com.unity.ml-agents/package.json and com.unity.ml-agents.extension/package.json to Unity package manager as mentioned in the guide.

After this, your setup should be done!

## This is the main guide I followed for this specific example [here](https://unity-technologies.github.io/ml-agents/Learning-Environment-Create-New/)

### Pretrained model can be found under Assets/RollerBall.onnx
