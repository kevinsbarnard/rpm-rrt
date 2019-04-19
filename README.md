# RPM-RRT
***Authors**: Kevin Barnard, Harrison Oest, Michael Le*

RRT algorithm implementation for CSCI 498: Robot Planning & Manipulation

## Instructions
The program has two primary modes of operation: **evaluate** and **visualize**. 

Three algorithms are implemented:

- RRT
- RRT-multi
- RRT-connect

### evaluate
The **evaluate** mode is designated for data collection in parallel for the three algorithms.

Usage:
```bash
python evaluate.py [epsilon] [limit] [batch] [n_obstacles]
```
For help with the options, run `evaluate.py` with the `-h` or `--help` option.

### visualize
The **visualize** mode is designated for visualizing how the algorithms perform on a predefined space of obstacles.

Usage:
```bash
python visualize.py
```
There are no options for `visualize.py`.

## Dependencies
- [Python 3.7.3](https://www.python.org/)
- [matplotlib 3.0.3](https://matplotlib.org/)