# scripts/python

## kdl-from-csv

### Install dependecies
- Requires Python [KDL](http://robots.uc3m.es/gitbook-installation-guides/install-kdl.html) (PyKDL)
- Defaults hard-coded from https://github.com/roboticslab-uc3m/teo-developer-manual

### Help
```bash
python kdl-from-csv.py -h
```

### Run
```bash
python kdl-from-csv.py --dhFileName /home/yo/repos/teo-developer-manual/csv/dh-root-rightLeg.csv
```

## roboview-from-csv
Provides a kinematic chain description for [roboview](https://github.com/arcoslab/roboview), given a `.csv` file.

### Install dependecies
- [roboview](https://github.com/arcoslab/roboview)
   - At time of writing, best use `vtk6` branch of <https://github.com/jgvictores/roboview> fork
   - Requires [Python YARP](http://robots.uc3m.es/gitbook-installation-guides/install-yarp.html#install-bindings) and Python [KDL](http://robots.uc3m.es/gitbook-installation-guides/install-kdl.html) (PyKDL)
- Examples hard-coded from https://github.com/roboticslab-uc3m/teo-developer-manual

### Run
```bash
yarp server &
./roboview roboview-from-csv.py
```
