# scripts/python

## roboview-from-csv

### Install dependecies
- https://github.com/arcoslab/roboview
   - Fork https://github.com/jgvictores/roboview branch `vtk6` at time of writing
   - Requires [Python YARP](http://robots.uc3m.es/gitbook-installation-guides/install-yarp.html#install-bindings) and Python [KDL](http://robots.uc3m.es/gitbook-installation-guides/install-kdl.html) (PyKDL)
- Examples hard-coded from https://github.com/roboticslab-uc3m/teo-developer-manual

### Run
```bash
yarp server &
./roboview roboview-from-csv.py
```
