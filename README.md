# Download
Download urdf one by one: https://sapien.ucsd.edu/browse

# Inertial Generation
Inertia is not included in raw urdf. `inertia_generator.py` can do this automatically. THere are several parameters to set. But you can use the default one fo rthe first time. After this, you will get `mobility_new.urdf` for each object. An exampel is also provided here.

# Load object
`loader.py` provide a function to load object and analyze sbbox form metadata. Please refer to `main.py` for a toy example.