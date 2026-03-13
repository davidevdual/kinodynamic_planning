# kinodynamic_planning
Tutorial about kinodynamic planning

## Installation for Visual Studio Community 2022

- Install miniconda3
- Do not add the conda environment to the PATH variable
- Open the Anaconda prompt from the Search bar in Windows
- Create the conda virtual environment kinodynamic_planning_venv from the Anaconda prompt. kinodynamic_planning_venv uses Python 3.13:

```bash
conda create -n kinodynamic_planning_venv python=3.13
```

- Activate the environment kinodynamic_planning_venv

```bash
conda activate kinodynamic_planning_venv
```

- Look for the python installation inside kinodynamic_planning_venv by writing in the command prompt:

```bash
where python
```

- Copy the output

```bash
C:\Users\DNCM\miniconda3\envs\kinodynamic_planning_venv\python.exe
```

- Open Visual Studio Community 2022 and click on "Add Environment"
- Link to the kinodynamic_planning_venv virtual environment. Paste the python path previously copied

- Install pybullet in the conda environment:

```bash
pip install pybullet
```

## Files

- test_simplegrasp_pstctrl.py: Move the Franka arm to the object and grasp the object. The joint positions are pre-defined.