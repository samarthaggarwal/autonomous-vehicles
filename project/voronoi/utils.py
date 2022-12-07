import numpy as np

def readfile(filename):
    """
    returns the entire file contents of a txt file
    """
    f = open(filename, "r")
    txt = f.read()
    f.close()
    return txt

def readgrid(filename):
    """
    reads the grid of 0/1 from a txt file and returns the corresponding numpy array
    """
    txt = readfile(filename)
    txt = txt.split('\n')
    m, n = len(txt), len(txt[0])
    grid = np.zeros((m,n), dtype=int)
    for i, line in enumerate(txt):
        for j, ch in enumerate(line):
            if ch=='1':
                grid[i][j] = 1
    return grid